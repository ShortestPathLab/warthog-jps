#ifndef JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR_H
#define JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR_H

// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011,
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//

#include <jps/forward.h>
#include <warthog/domain/gridmap.h>
#include <array>
#include <memory>

namespace jps::jump
{

namespace details
{

/// @brief 
/// @tparam East Jump east (true) or west (false)
/// @param map 
/// @param node 
/// @param goal 
/// @return 
template <bool East>
uint32_t jump_point_online_hori(const ::warthog::domain::gridmap& map, jps_id node, jps_id goal)
{
	// read tiles from the grid:
	// - along the row of node_id
	// - from the row above node_id
	// - from the row below node_id
	// NB: the jump direction (here, EAST) corresponds to moving from the
	// low bit of the tileset and towards the high bit
	auto nei_slider = map.get_neighbours_slider(node);
	nei_slider.adj_bytes(East ? -1 : -6); // current location is 1 byte from boundrary
		// bit position 0 = highest bit order, reverse of normal
	nei_slider.width8_bits = East ? nei_slider.width8_bits - 8 : 15 - nei_slider.width8_bits;

	// order going east is stored as least significant bit to most significant bit

	while(true)
	{
		std::array<uint64_t, 3 neis = nei_slider.get_neighbours_64bit_le();
		assert(nei_slider.width8_bits < 16);
		uint64_t tmp = East ? ~(~0ull << nei_slider.width8_bits)
			: ~(~0ull >> nei_slider.width8_bits);
		// shift above and below 2 points east
		// mask out to trav(1) before node
		neis[0] |= tmp;
		neis[1] |= tmp;
		neis[2] |= tmp;
		//   v  & will make this the least sig bit, and is where the jump point is located
		// 10100011 : 0
		// 10111000 : <<1~
		if constexpr (East) {
			tmp = ((~neis[1] << 1) & neis[1]) // above row: block(zero) trailing trav(one)
				| ((~neis[2] << 1) & neis[2]); // below row: block(zero) trailing trav(one)
		} else {
			tmp = ((~neis[1] >> 1) & neis[1]) // above row: block(zero) trailing trav(one)
				| ((~neis[2] >> 1) & neis[2]); // below row: block(zero) trailing trav(one)
		}
		tmp = tmp | ~neis[0];
		if (tmp) {
			int stop_pos = East ? std::countr_zero(tmp) : std::countl_zero(tmp); // the location to stop at
			//  v is blocker location, prune unless target is present
			// 10111111
			// dead end takes president as jump point can't pass a blocker
			jump_count += static_cast<uint32_t>(stop_pos) - nei_slider.width8_bits;
			if ( East ?
				(tmp & (1u << stop_pos)) :
				(tmp & (std::numeric_limits<uint64_t>::min() >> stop_pos))
				) // deadend
			{
				// jump_count = blocker_pos, jump_count-1 = true stop_pos
				uint32_t goal_jump = East ? static_cast<uint32_t>(goal) - static_cast<uint32_t>(node)
					: static_cast<uint32_t>(node) - static_cast<uint32_t>(goal);
				jump_count = goal_jump < jump_count ? goal_jump : 0;
			}
			return jump_count;
		}

		// failed, goto next 56 bits
		jump_count += 63 - nei_slider.width8_bits;
		nei_slider.adj_bytes(East ? 7 : -7);
		nei_slider.width8_bits = 7;
	}
}

}

template <JpsFeature Feature = JpsFeature::DEFAULT>
class jump_point_online
{
public:
	using gridmap = warthog::domain::gridmap;
	online_jump_point_locator();
	online_jump_point_locator(const gridmap& map);
	~online_jump_point_locator();

	void set_map(const gridmap& map)
	{
		map_ = &map;
		rotate_map_ = create_rotate_(map_);
	}

	void
	jump(
	    direction d, jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);

	size_t
	mem()
	{
		return sizeof(this) + rmap_->mem();
	}

protected:

	static uint32_t jump_east(const gridmap& map, jps_id node, jps_id goal);
	static uint32_t jump_west(const gridmap& map, jps_id node, jps_id goal);

	template <direction D>
	static uint32_t jump_inter_cardinal(const gridmap& map, jps_id node, jps_id goal);

	inline jps_id
	map_id_to_rmap_id(jps_id mapid)
	{
		if(mapid.is_none()) { return jps_id::none(); }

		uint32_t x, y;
		uint32_t rx, ry;
		map_->to_unpadded_xy(mapid, x, y);
		ry = x;
		rx = map_->header_height() - y - 1;
		return jps_id(rmap_->to_padded_id_from_unpadded(rx, ry));
	}

	inline jps_id
	rmap_id_to_map_id(jps_id rmapid)
	{
		if(rmapid.is_none()) { return jps_id::none(); }

		uint32_t x, y;
		uint32_t rx, ry;
		rmap_->to_unpadded_xy(rmapid, rx, ry);
		x = ry;
		y = rmap_->header_width() - rx - 1;
		return jps_id(map_->to_padded_id_from_unpadded(x, y));
	}

protected:
	/**
	 * Rotate 90 clockwise
	 * NORTH -> EAST
	 * EAST -> SOUTH
	 * SOUTH -> WEST
	 * WEST -> NORTH
	 * NORTHEAST -> SOUTHEAST
	 * SOUTHEAST -> SOUTHWEST
	 * SOUTHWEST -> NORTHWEST
	 * NORTHWEST -> NORTHEAST
	 * 
	 * unpadded (x,y) -> (y, Rh-1-x)
	 */
	std::unique_ptr<gridmap>
	create_rotate_(const gridmap& orig)
	{
		const uint32_t maph = map_->header_height();
		const uint32_t mapw = map_->header_width();
		const uint32_t rmaph = mapw - 1;
		const uint32_t rmapw = maph;
		auto tmap = std::make_unique<gridmap>(rmaph + 1, rmapw);

		for(uint32_t x = 0; x < mapw; x++)
		{
			for(uint32_t y = 0; y < maph; y++)
			{
				bool label
					= map_->get_label(map_->to_padded_id_from_unpadded(x, y));
				uint32_t rx = y;
				uint32_t ry = rmaph - x;
				tmap->set_label(tmap->to_padded_id_from_unpadded(rx, ry), label);
			}
		}
		return tmap;
	}

protected:
	const gridmap* map_ = {};
	std::unique_ptr<gridmap> rotate_map_;
	uint32_t map_width_ = 0;
	uint32_t rotate_map_width_ = 0;
	point transpose_adj_ = {};
};


template <JpsFeature Feature>
auto
jump_point_online<Feature>::create_rotate_(const gridmap& orig) -> std::unique_ptr<gridmap>
{
	const uint32_t maph = map_->header_height();
	const uint32_t mapw = map_->header_width();
	const uint32_t rmaph = mapw - 1;
	const uint32_t rmapw = maph;
	auto tmap = std::make_unique<gridmap>(rmaph + 1, rmapw);

	for(uint32_t x = 0; x < mapw; x++)
	{
		for(uint32_t y = 0; y < maph; y++)
		{
			bool label
				= map_->get_label(map_->to_padded_id_from_unpadded(x, y));
			uint32_t rx = y;
			uint32_t ry = rmaph - x;
			tmap->set_label(tmap->to_padded_id_from_unpadded(rx, ry), label);
		}
	}
	return tmap;
}

template <JpsFeature Feature>
uint32_t jump_point_online<Feature>::jump_east(const gridmap& map, jps_id node, jps_id goal)
{
	return details::jump_point_online_hori<true>(map, node, goal);
}

template <JpsFeature Feature>
uint32_t jump_point_online<Feature>::jump_west(const gridmap& map, jps_id node, jps_id goal)
{
	return details::jump_point_online_hori<false>(map, node, goal);
}

template <direction D>
uint32_t jump_inter_cardinal(const gridmap& map, jps_id node, jps_id goal)
{
	/*
	map:
	NW N NE
	 W   E
	SW S SE

	rmap = rotate 90 CW:
	SW W NW
	 S   N
	SE E NE

	SE = jump_intercardinal_pos(M0=map,M1=rmap)
	map:  x+=1, y+=1, pos += mapW+1
	rmap: x-=1, y+=1, pos += rmapW-1
	jump_south = rmap: jump_west(M1), (x,y+r)
	jump_east = map: jump_east(M0), (x+r,y)

	NE = jump_intercardinal_pos(M0=rmap,M1=map)
	rmap: x+=1, y+=1, pos += rmapW+1
	map:  x+=1, y-=1, pos -= mapW-1
	jump_east = map: jump_east(M1), (x+r,y)
	jump_north = rmap: jump_east(M0), (x,y-r)

	NW = jump_intercardinal_neg(M0=map,M1=rmap)
	map:  x-=1, y-=1, pos -= mapW+1
	rmap: x+=1, y-=1, pos -= rmapW-1
	jump_north = rmap: jump_east(M1), (x,y-r)
	jump_west = map: jump_west(M0), (x-r,y)

	SW = jump_intercardinal_neg(M0=rmap,M1=map)
	rmap: x-=1, y-=1, pos -= rmapW+1
	map:  x-=1, y+=1, pos += mapW-1
	jump_south = rmap: jump_west(M0), (x,y+r)
	jump_west = map: jump_west(M1), (x-r,y)
	*/
}

}

#endif // JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR_H
