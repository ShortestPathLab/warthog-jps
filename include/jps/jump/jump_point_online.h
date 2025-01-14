#ifndef JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR_H
#define JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR_H

// online_jump_point_locator.h
//
// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011,
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//
// @author: dharabor
// @created: 03/09/2012
//

#include <jps/forward.h>
#include <warthog/domain/gridmap.h>
#include <array>
#include <memory>

namespace jps::jump
{

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
		transpose_ = create_transpose_(map_);
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

	static point jump_east(const gridmap& map, jps_id node, jps_id goal);
	static point jump_west(const gridmap& map, jps_id node, jps_id goal);

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
point jump_point_online<Feature>::jump_east(const gridmap& map, jps_id node, jps_id goal)
{
	std::array<uint64_t, 3> neis{};
	bool deadend = false;

	// read tiles from the grid:
	// - along the row of node_id
	// - from the row above node_id
	// - from the row below node_id
	// NB: the jump direction (here, EAST) corresponds to moving from the
	// low bit of the tileset and towards the high bit
	auto nei_slider = map.get_neighbours_slider(node);
	nei_slider.adj_bytes(-1); // current location is within byte 1 for detecting

	// order going east is stored as least significant bit to most significant bit

	while(true)
	{
		neis = nei_slider.get_neighbours_64bit_le();
		// example version
		// ~neis[1] | (~neis[0] << 1) | (~neis[2] << 1)
		// the lsb denotes a turning point
		// while lsb(~neis[1]) is a dead end
		// as we need 1 bit shift for upper and lower row, we start in byte 1 and shift 7 bytes
	}

	// look for tiles with forced neighbours in the rows above and below
	// A forced neighbour can be identified as a non-obstacle tile that
	// follows immediately  after an obstacle tile.
	// we ignore forced tiles which are at offsets >= bit_offset
	// (i.e., all tiles in {WEST of, above, below} the current location)
	uint64_t forced_bits = (~neis[0] << 1) & neis[0];
	forced_bits |= (~neis[2] << 1) & neis[2];
	forced_bits &= ~((1LL << bit_offset) | ((1LL << bit_offset) - 1));

	// look for obstacles tiles in the current row
	// we ignore obstacles at offsets > bit_offset
	uint64_t deadend_bits = ~neis[1];
	deadend_bits &= ~((1LL << bit_offset) - 1);

	// stop jumping if any forced or deadend locations are found
	uint64_t stop_bits = (forced_bits | deadend_bits);
	if(stop_bits)
	{
		// figure out how far we jumped (we count trailing zeroes)
		// we then subtract -1 because we want to know how many
		// steps from the bit offset to the stop bit
		int stop_pos = __builtin_ctzll(stop_bits);
		uint32_t num_steps = (stop_pos - bit_offset);

		// don't jump over the target
		uint32_t goal_dist = goal_id.id - node_id.id;
		if(num_steps > goal_dist)
		{
			jumpnode_id = goal_id;
			jumpcost = goal_dist;
			return;
		}

		// did we reach a jump point or a dead-end?
		deadend = (deadend_bits & (1LL << stop_pos));
		if(deadend)
		{
			jumpcost = num_steps - (1 && num_steps);
			jumpnode_id = jps_id::none();
			return;
		}

		jumpnode_id = jps_id(node_id.id + num_steps);
		jumpcost = num_steps;
		return;
	}

	// keep jumping. the procedure below is implemented
	// similarly to the above. but now the stride is a
	// fixed 64bit and the jumps are word-aligned.
	jumpnode_id = jps_id(node_id.id + 64 - bit_offset);
	while(true)
	{
		// we need to forced neighbours might occur across
		// 64bit boundaries. to check for these we keep the
		// high-byte of the previous set of neighbours
		uint64_t high_ra = neis[0] >> 63;
		uint64_t high_rb = neis[2] >> 63;

		// read next 64bit set of tile data
		mymap->get_neighbours_64bit(jumpnode_id, neis);

		// identify forced neighbours and deadend tiles.
		uint64_t forced_bits = ~((neis[0] << 1) | high_ra) & neis[0];
		forced_bits |= ~((neis[2] << 1) | high_rb) & neis[2];
		uint64_t deadend_bits = ~neis[1];

		// stop if we found any forced or dead-end tiles
		uint64_t stop_bits = (forced_bits | deadend_bits);
		if(stop_bits)
		{
			int stop_pos = __builtin_ctzll(stop_bits);
			jumpnode_id.id += stop_pos;
			deadend = deadend_bits & (1LL << stop_pos);
			break;
		}
		jumpnode_id.id += 64;
	}

	// figure out how far we jumped
	uint32_t num_steps = jumpnode_id.id - node_id.id;

	// don't jump over the target
	uint32_t goal_dist = goal_id.id - node_id.id;
	if(num_steps > goal_dist)
	{
		jumpnode_id = goal_id;
		jumpcost = goal_dist;
		return;
	}

	// did we hit a dead-end?
	if(deadend)
	{
		// in this case we want to return the number of steps to
		// the last traversable tile (not to the obstacle)
		// need -1 to fix it.
		num_steps -= (1 && num_steps);
		jumpnode_id = jps_id::none();
	}

	// return the number of steps to reach the jump point or deadend
	jumpcost = num_steps;
}

}

#endif // JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR_H
