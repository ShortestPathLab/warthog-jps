#ifndef JPS_JUMP_JUMP_POINT_ONLINE_H
#define JPS_JUMP_JUMP_POINT_ONLINE_H

// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011,
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//

#include <array>
#include <jps/forward.h>
#include <memory>
#include <warthog/domain/gridmap.h>

namespace jps::jump
{

namespace details
{

/// @brief
/// @tparam East Jump east (true) or west (false)
/// @param map a map of the grid, only the width parameter is required
/// @param node the starting location
/// @param goal the goal location
/// @return
template<bool East>
int32_t
jump_point_online_hori(
    ::warthog::domain::gridmap::bittable map, uint32_t node, uint32_t goal)
{
	// read tiles from the grid:
	// - along the row of node_id
	// - from the row above node_id
	// - from the row below node_id
	// NB: the jump direction corresponds to moving from the
	// low bit of the tileset and towards the high bit (EAST)
	// or high bit to low bit (WEST)
	auto nei_slider
	    = ::warthog::domain::gridmap_slider::from_bittable(map, pad_id{node});
	nei_slider.adj_bytes(
	    East ? -1 : -6); // current location is 1 byte from boundrary
	// width8_bits is how many bits in on word current node is at, from lsb
	// EAST and from msb WEST
	nei_slider.width8_bits
	    = East ? nei_slider.width8_bits + 8 : 15 - nei_slider.width8_bits;
	// 15 - width8_bits == 63 - (width8_bits + 6 * 8)
	int32_t jump_count = 0;

	// order going east is stored as least significant bit to most significant
	// bit

	while(true)
	{
		std::array<uint64_t, 3> neis = nei_slider.get_neighbours_64bit_le();
		assert(nei_slider.width8_bits < 16);
		uint64_t tmp = East ? ~(~0ull << nei_slider.width8_bits)
		                    : ~(~0ull >> nei_slider.width8_bits);
		// shift above and below 2 points east
		// mask out to trav(1) before node location
		neis[0] |= tmp;
		neis[1] |= tmp;
		neis[2] |= tmp;
		// find first jump point, is +1 location past blocker above or below
		if constexpr(East)
		{
			tmp = ((~neis[1] << 1)
			       & neis[1]) // above row: block(zero) trailing trav(one)
			    | ((~neis[2] << 1)
			       & neis[2]); // below row: block(zero) trailing trav(one)
		}
		else
		{
			tmp = ((~neis[1] >> 1)
			       & neis[1]) // above row: block(zero) trailing trav(one)
			    | ((~neis[2] >> 1)
			       & neis[2]); // below row: block(zero) trailing trav(one)
		}
		// append for dead-end check
		tmp = tmp | ~neis[0];
		if(tmp)
		{
			int stop_pos = East
			    ? std::countr_zero(tmp)
			    : std::countl_zero(tmp); // the location to stop at
			//  v is blocker location, prune unless target is present
			// 10111111
			// dead end takes president as jump point can't pass a blocker
			jump_count += static_cast<int32_t>(stop_pos)
			    - static_cast<int32_t>(nei_slider.width8_bits);
			uint32_t goal_jump = East ? goal - node : node - goal;
			// if blocked: pos + jump_count = first block
			//  otherwise: jump_count = trav cell after turn (jump point
			//  location)
			// check for goal with goal_jump (dist) <= jump_count, as if < than
			// goal is reachable, if equal then trav pos is the goal if
			// greater, than goal is further or another row or behind (as
			// unsigned)
			assert(jump_count >= 0);
			// must be checked as unsigned for:
			// 1. goal.is_none(): will not fit int32_t
			// 2. underflow means goal is in opposite direction, desirable
			if(goal_jump <= static_cast<uint32_t>(jump_count))
			{
				// goal reached
				jump_count = static_cast<int32_t>(goal_jump);
			}
			else if(
			    East ? !(neis[0] & (static_cast<uint64_t>(1) << stop_pos))
			         : !(neis[0]
			             & (static_cast<uint64_t>(
			                    std::numeric_limits<int64_t>::min())
			                >> stop_pos))) // deadend
			{
				// deadend, return negative jump
				assert(jump_count > 0);
				jump_count = -(jump_count - 1);
			}
			return jump_count;
		}

		// failed, goto next 56 bits
		jump_count += 63 - nei_slider.width8_bits;
		nei_slider.adj_bytes(East ? 7 : -7);
		nei_slider.width8_bits = 7;
	}
}

} // namespace details

struct intercardinal_jump_result
{
	jps_id node;
	jps_rid rnode;
	uint32_t dist;
};
template<direction D>
struct IntercardinalWalker
{
	static_assert(
	    D == NORTHEAST || D == NORTHWEST || D == SOUTHEAST || D == SOUTHWEST,
	    "Must be intercardinal direction");
	union LongJumpRes
	{
		uint32_t dist[2]; /// distance hori/vert of D a jump is valid to
		uint64_t joint;
	};
	using map_type = ::warthog::domain::gridmap::bitarray;
	/// @brief map and rmap (as bit array for small memory size)
	map_type map[2];
	/// @brief location of current node on map and rmap
	uint32_t node_at[2];
	/// @brief map and rmap value to adjust node_at for each row
	int32_t adj_width[2];
	/// @brief map and rmap goal locations
	uint32_t goal[2];
	/// @brief row scan
	union
	{
		uint8_t row_[2]; ///< stores 3 bits at node_at[0]+-1, 0=prev,
		                 ///< 1=current; high order bits 3..7 are not zero'd
		uint16_t row_i_;
	};

	/// @brief convert map width to a map adj_width variable suited to
	/// intercardinal D2
	template<direction D2 = D>
	static constexpr int32_t
	to_map_adj_width(uint32_t width) noexcept
	{
		static_assert(
		    D2 == NORTHEAST || D2 == NORTHWEST || D2 == SOUTHEAST
		        || D2 == SOUTHWEST,
		    "Must be intercardinal direction");
		assert(width > 0);
		if constexpr(D2 == NORTHEAST)
		{
			return -static_cast<int32_t>(width - 1); // - (mapW-1)
		}
		else if constexpr(D2 == SOUTHEAST)
		{
			return static_cast<int32_t>(width + 1); // + (mapW+1)
		}
		else if constexpr(D2 == SOUTHWEST)
		{
			return static_cast<int32_t>(width - 1); // + (mapW-1)
		}
		else
		{                                            // NORTHWEST
			return -static_cast<int32_t>(width + 1); // - (mapW+1)
		}
	}
	/// @brief convert rmap width to a rmap adj_width variable suited to
	/// intercardinal D2
	template<direction D2 = D>
	static constexpr int32_t
	to_rmap_adj_width(uint32_t width) noexcept
	{
		return to_map_adj_width<dir_cw(D2)>(width);
	}

	/// @brief convert map adj_width to map width, reciprocal to
	/// to_map_adj_width
	template<direction D2 = D>
	static constexpr uint32_t
	from_map_adj_width(int32_t adj_width) noexcept
	{
		static_assert(
		    D2 == NORTHEAST || D2 == NORTHWEST || D2 == SOUTHEAST
		        || D2 == SOUTHWEST,
		    "Must be intercardinal direction");
		if constexpr(D2 == NORTHEAST)
		{
			return static_cast<uint32_t>(-adj_width + 1);
		}
		else if constexpr(D2 == SOUTHEAST)
		{
			return static_cast<uint32_t>(adj_width - 1);
		}
		else if constexpr(D2 == SOUTHWEST)
		{
			return static_cast<uint32_t>(adj_width + 1);
		}
		else
		{ // NORTHWEST
			return static_cast<uint32_t>(-adj_width - 1);
		}
	}
	/// @brief convert rmap adj_width to rmap width, reciprocal to
	/// to_rmap_adj_width
	template<direction D2 = D>
	static constexpr uint32_t
	from_rmap_adj_width(int32_t adj_width) noexcept
	{
		return from_map_adj_width<dir_cw(D2)>(adj_width);
	}

	/// @brief set map width`
	void
	map_width(uint32_t width) noexcept
	{
		adj_width[0] = to_map_adj_width(width);
	}
	/// @brief get map width
	uint32_t
	map_width() const noexcept
	{
		return from_map_adj_width(adj_width[0]);
	}
	/// @brief set rmap width
	void
	rmap_width(uint32_t width) noexcept
	{
		adj_width[1] = to_rmap_adj_width(width);
	}
	/// @brief get rmap width
	uint32_t
	rmap_width() const noexcept
	{
		return from_rmap_adj_width(adj_width[1]);
	}

	static uint32_t
	jump_east(map_type map, uint32_t width, uint32_t node, uint32_t goal)
	{
		int32_t d = details::jump_point_online_hori<true>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node, goal);
		// for LongJumpRes, must return 0 for deadend
		return d >= 0 ? static_cast<uint32_t>(d) : 0;
	}
	static uint32_t
	jump_west(map_type map, uint32_t width, uint32_t node, uint32_t goal)
	{
		int32_t d = details::jump_point_online_hori<false>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node, goal);
		// for LongJumpRes, must return 0 for deadend
		return d >= 0 ? static_cast<uint32_t>(d) : 0;
	}
	uint32_t
	jump_hori()
	{
		if constexpr(D == NORTHEAST)
		{
			return jump_east(map[0], map_width(), node_at[0], goal[0]); // east
		}
		else if constexpr(D == SOUTHEAST)
		{
			return jump_east(map[0], map_width(), node_at[0], goal[0]); // east
		}
		else if constexpr(D == SOUTHWEST)
		{
			return jump_west(map[0], map_width(), node_at[0], goal[0]); // west
		}
		else if constexpr(D == NORTHWEST)
		{
			return jump_west(map[0], map_width(), node_at[0], goal[0]); // west
		}
	}
	uint32_t
	jump_vert()
	{
		if constexpr(D == NORTHEAST)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1], goal[1]); // north
		}
		else if constexpr(D == SOUTHEAST)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1], goal[1]); // south
		}
		else if constexpr(D == SOUTHWEST)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1], goal[1]); // south
		}
		else if constexpr(D == NORTHWEST)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1], goal[1]); // north
		}
	}

	void
	next_index() noexcept
	{
		node_at[0] = static_cast<uint32_t>(
		    static_cast<int32_t>(node_at[0]) + adj_width[0]);
		node_at[1] = static_cast<uint32_t>(
		    static_cast<int32_t>(node_at[1]) + adj_width[1]);
	}

	/// @brief return get node_at[0]-1..node_at[0]+1 bits. CAUTION: return
	/// bits 3..7 may not all be 0.
	uint8_t
	get_row() const noexcept
	{
		return static_cast<uint8_t>(
		    map[0].get_span<3>(pad_id{node_at[0] - 1}));
	}
	/// @brief call for first row, then call next_row
	void
	first_row() noexcept
	{
		row_[1] = get_row();
	}
	/// @brief update index to next row and update
	void
	next_row() noexcept
	{
		next_index();
		row_[0] = row_[1];
		row_[1] = get_row();
	}
	/// @brief get node id for location node + dist(EAST/WEST of D)
	jps_id
	adj_hori(uint32_t node, uint32_t dist) const noexcept
	{
		if constexpr(D == NORTHEAST || D == SOUTHEAST)
		{
			return jps_id{node + dist};
		}
		else { return jps_id{node - dist}; }
	}
	/// @brief get node id for location node + dist(NORTH/SOUTH of D)
	jps_id
	adj_vert(uint32_t node, uint32_t dist) const noexcept
	{
		if constexpr(D == NORTHEAST || D == SOUTHEAST)
		{
			return jps_id{node + (adj_width[0] - 1) * dist};
		}
		else { return jps_id{node + (adj_width[0] + 1) * dist}; }
	}
	/// @brief the current locations row is a valid intercardinal move (i.e.
	/// 2x2 is free)
	bool
	valid_row() const noexcept
	{
		// east | west differernce
		// north/south does not make a difference
		if constexpr(D == NORTHEAST || D == SOUTHEAST)
		{
			// we want from grid
			// .xx  == row[0] = 0bxx.
			//  xx. == row[1] = 0b.xx
			// all[x] = 1
			constexpr uint16_t mask
			    = std::endian::native == std::endian::little
			    ? 0b0000'0011'0000'0110
			    : 0b0000'0110'0000'0011;
			return (row_i_ & mask) == mask;
		}
		else
		{
			// we want from grid
			//  xx. == row[0] = 0b.xx
			// .xx  == row[1] = 0bxx.
			// all[x] = 1
			constexpr uint16_t mask
			    = std::endian::native == std::endian::little
			    ? 0b0000'0110'0000'0011
			    : 0b0000'0011'0000'0110;
			return (row_i_ & mask) == mask;
		}
	}
	// {hori,vert} of jump hori/vert of D, from node_at.
	LongJumpRes
	long_jump()
	{
		return {jump_hori(), jump_vert()};
	}
};

template<JpsFeature Feature = JpsFeature::DEFAULT>
class jump_point_online
{
public:
	using gridmap  = warthog::domain::gridmap;
	using map_type = gridmap::bittable;
	jump_point_online();
	jump_point_online(gridmap* map)
	{
		if(map != nullptr) set_map(*map);
	}
	~jump_point_online() = default;

	static consteval bool
	feature_prune_intercardinal() noexcept
	{
		return (static_cast<uint32_t>(Feature)
		        & static_cast<uint32_t>(JpsFeature::PRUNE_INTERCARDINAL))
		    != 0;
	}
	static consteval bool
	feature_store_cardinal() noexcept
	{
		return (static_cast<uint32_t>(Feature)
		        & static_cast<uint32_t>(JpsFeature::STORE_CARDINAL_JUMP))
		    != 0;
	}

	void
	set_map(gridmap& map);
	void
	set_goal(jps_id goal_id) noexcept;
	void
	set_goal(point goal_id) noexcept;

	/// @brief avoid modifying these grids accidentally
	gridmap::bittable
	get_map() const noexcept
	{
		return map_;
	}
	/// @brief care should be taken to avoid modifying these grids
	gridmap::bittable
	get_rmap() noexcept
	{
		return rmap_;
	}

	/**
	 * @returns pair first: steps to reach jump point, second: id
	 * of jump point. deadend returns negative steps
	 * (include 0) to reach before blocker.
	 */
	std::pair<int32_t, jps_id>
	jump_cardinal(direction d, jps_id node_id, jps_rid rnode_id);
	intercardinal_jump_result
	jump_intercardinal(
	    direction d, jps_id node_id, jps_rid rnode_id, jps_id* result_node,
	    cost_t* result_cost, uint32_t result_size = 0);

	int32_t
	jump_north(jps_rid rnode)
	{
		return jump_east(rmap_, rnode.id, rgoal_.id);
	}
	int32_t
	jump_east(jps_id node)
	{
		return jump_east(map_, node.id, goal_.id);
	}
	int32_t
	jump_south(jps_rid rnode)
	{
		return jump_west(rmap_, rnode.id, rgoal_.id);
	}
	int32_t
	jump_west(jps_id node)
	{
		return jump_west(map_, node.id, goal_.id);
	}
	intercardinal_jump_result
	jump_northeast(
	    jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost,
	    uint32_t result_size = 0)
	{
		return jump_intercardinal<NORTHEAST>(
		    node, rnode, result_node, result_cost, result_size);
	}
	intercardinal_jump_result
	jump_southeast(
	    jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost,
	    uint32_t result_size)
	{
		return jump_intercardinal<SOUTHEAST>(
		    node, rnode, result_node, result_cost, result_size);
	}
	intercardinal_jump_result
	jump_southwest(
	    jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost,
	    uint32_t result_size)
	{
		return jump_intercardinal<SOUTHWEST>(
		    node, rnode, result_node, result_cost, result_size);
	}
	intercardinal_jump_result
	jump_northwest(
	    jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost,
	    uint32_t result_size)
	{
		return jump_intercardinal<NORTHWEST>(
		    node, rnode, result_node, result_cost, result_size);
	}

	size_t
	mem()
	{
		return sizeof(this) + (rmap_data_ ? rmap_data_->mem() : 0);
	}

	point
	point_to_rpoint(point p) const noexcept
	{
		return {
		    static_cast<uint16_t>(map_unpad_height_m1_ - p.y),
		    static_cast<uint16_t>(p.x)};
	}
	point
	rpoint_to_point(point p) const noexcept
	{
		return {
		    static_cast<uint16_t>(p.y),
		    static_cast<uint16_t>(map_unpad_height_m1_ - p.x)};
	}
	jps_id
	point_to_id(point p) const noexcept
	{
		return jps_id{
		    static_cast<jps_id::id_type>(p.y + gridmap::PADDED_ROWS)
		        * map_.width()
		    + static_cast<jps_id::id_type>(p.x)};
	}
	jps_rid
	rpoint_to_rid(point p) const noexcept
	{
		return jps_rid{
		    static_cast<jps_rid::id_type>(p.y + gridmap::PADDED_ROWS)
		        * rmap_.width()
		    + static_cast<jps_rid::id_type>(p.x)};
	}
	point
	id_to_point(jps_id p) const noexcept
	{
		return {
		    static_cast<uint16_t>(p.id % map_.width()),
		    static_cast<uint16_t>(p.id / map_.width() - gridmap::PADDED_ROWS)};
	}
	point
	rid_to_rpoint(jps_rid p) const noexcept
	{
		return {
		    static_cast<uint16_t>(p.id % rmap_.width()),
		    static_cast<uint16_t>(
		        p.id / rmap_.width() - gridmap::PADDED_ROWS)};
	}
	jps_rid
	id_to_rid(jps_id mapid)
	{
		assert(!mapid.is_none());
		return rpoint_to_rid(point_to_rpoint(id_to_point(mapid)));
	}
	jps_id
	rid_to_id(jps_rid mapid)
	{
		assert(!mapid.is_none());
		return point_to_id(rpoint_to_point(rid_to_rpoint(mapid)));
	}

protected:
	static int32_t
	jump_east(map_type map, uint32_t node, uint32_t goal);
	static int32_t
	jump_west(map_type map, uint32_t node, uint32_t goal);

	/**
	 * Jumps on the intercardinal.
	 * The result_* variables store the cardinal jump results (if enabled)
	 * result_count only used for PRUNE_INTERCARDINAL.
	 * result_* must be big enough to store:
	 * !PRUNE_INTERCARDINAL & !STORE_CARDINAL_JUMP => 0 (should be nullptr)
	 * !PRUNE_INTERCARDINAL & STORE_CARDINAL_JUMP => 2
	 * PRUNE_INTERCARDINAL => result_cost (min 4)
	 *
	 * if !PRUNE_INTERCARDINAL & STORE_CARDINAL_JUMP:
	 *   results[0] = east/west result or jps_id::none() if none
	 *   results[1] = north/south result or jps_id::none() if none
	 *
	 * The return intercardinal_jump_result is as follows:
	 * node: return end point id, or jps_id::none() if no more successors.
	 * rnode: return end rpoint id, or jps_rid::none() if no more successors.
	 * dist: !PRUNE_INTERCARDINAL => distance jumped (0 = no jump)
	 *        PRUNE_INTERCARDINAL => the number of elements pushed on the
	 * result node
	 */
	template<direction D>
	intercardinal_jump_result
	jump_intercardinal(
	    jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost,
	    uint32_t result_size = 0);

	// jps_id point_to_jps_id(point p) noexcept
	// {
	// 	return
	// }

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
	void
	create_rotate_(const gridmap& orig);

protected:
	std::unique_ptr<gridmap> rmap_data_;
	map_type map_  = {};
	map_type rmap_ = {};
	// uint32_t map_width_ = 0;
	// uint32_t rmap_width_ = 0;
	uint32_t map_unpad_height_m1_ = 0;
	jps_id goal_                  = {};
	jps_rid rgoal_                = {};
};

template<JpsFeature Feature>
void
jump_point_online<Feature>::set_map(gridmap& orig)
{
	map_ = orig;
	create_rotate_(orig);
}

template<JpsFeature Feature>
void
jump_point_online<Feature>::set_goal(jps_id p) noexcept
{
	goal_  = p;
	rgoal_ = id_to_rid(p);
}
template<JpsFeature Feature>
void
jump_point_online<Feature>::set_goal(point p) noexcept
{
	goal_  = point_to_id(p);
	rgoal_ = rpoint_to_rid(point_to_rpoint(p));
}

template<JpsFeature Feature>
void
jump_point_online<Feature>::create_rotate_(const gridmap& orig)
{
	const uint32_t maph = orig.header_height();
	const uint32_t mapw = orig.header_width();
	auto tmap           = std::make_unique<gridmap>(mapw, maph);

	for(uint32_t y = 0; y < maph; y++)
	{
		for(uint32_t x = 0; x < mapw; x++)
		{
			bool label = orig.get_label(orig.to_padded_id_from_unpadded(x, y));
			uint32_t rx = (maph - 1) - y;
			uint32_t ry = x;
			tmap->set_label(tmap->to_padded_id_from_unpadded(rx, ry), label);
		}
	}

	// set values
	rmap_data_ = std::move(tmap);
	rmap_      = *rmap_data_;
	// map_.width() = map_->width();
	// rmap_.width() = rmap_->width();
	map_unpad_height_m1_ = maph - 1;
}

template<JpsFeature Feature>
std::pair<int32_t, jps_id>
jump_point_online<Feature>::jump_cardinal(
    direction d, jps_id node_id, jps_rid rnode_id)
{
	std::pair<int32_t, jps_id> node;
	switch(d)
	{
	case NORTH:
		node.first     = jump_north(rnode_id);
		node.second.id = node_id.id
		    - map_.width()
		        * static_cast<jps_id::id_type>(std::abs(node.first));
		break;
	case SOUTH:
		node.first     = jump_south(rnode_id);
		node.second.id = node_id.id
		    + map_.width()
		        * static_cast<jps_id::id_type>(std::abs(node.first));
		break;
	case EAST:
		node.first = jump_east(node_id);
		node.second.id
		    = node_id.id + static_cast<jps_id::id_type>(std::abs(node.first));
		break;
	case WEST:
		node.first = jump_west(node_id);
		node.second.id
		    = node_id.id - static_cast<jps_id::id_type>(std::abs(node.first));
		break;
	default:
		assert(false);
		node = {0, node_id};
	}
	return node;
}
template<JpsFeature Feature>
intercardinal_jump_result
jump_point_online<Feature>::jump_intercardinal(
    direction d, jps_id node_id, jps_rid rnode_id, jps_id* result_node,
    cost_t* result_cost, uint32_t result_size)
{
	intercardinal_jump_result node;
	switch(d)
	{
	case NORTHEAST:
		node = jump_intercardinal<NORTHEAST>(
		    node_id, rnode_id, result_node, result_cost, result_size);
		break;
	case NORTHWEST:
		node = jump_intercardinal<NORTHWEST>(
		    node_id, rnode_id, result_node, result_cost, result_size);
		break;
	case SOUTHEAST:
		node = jump_intercardinal<SOUTHEAST>(
		    node_id, rnode_id, result_node, result_cost, result_size);
		break;
	case SOUTHWEST:
		node = jump_intercardinal<SOUTHWEST>(
		    node_id, rnode_id, result_node, result_cost, result_size);
		break;
	default:
		assert(false);
		node = {jps_id::none(), jps_rid::none(), 0};
	}
	return node;
}

template<JpsFeature Feature>
int32_t
jump_point_online<Feature>::jump_east(
    map_type map, uint32_t node, uint32_t goal)
{
	return details::jump_point_online_hori<true>(map, node, goal);
}

template<JpsFeature Feature>
int32_t
jump_point_online<Feature>::jump_west(
    map_type map, uint32_t node, uint32_t goal)
{
	return details::jump_point_online_hori<false>(map, node, goal);
}

template<JpsFeature Feature>
template<direction D>
intercardinal_jump_result
jump_point_online<Feature>::jump_intercardinal(
    jps_id node, jps_rid rnode, jps_id* result_node [[maybe_unused]],
    cost_t* result_cost [[maybe_unused]],
    uint32_t result_size [[maybe_unused]])
{
	static_assert(
	    D == NORTHEAST || D == NORTHWEST || D == SOUTHEAST || D == SOUTHWEST,
	    "D must be inter-cardinal.");

	assert(!node.is_none() && !rnode.is_none());

	// precondition
	assert(
	    !(feature_prune_intercardinal()
	      || feature_store_cardinal()) // both not enabled = fine
	    || (result_node != nullptr
	        && result_cost != nullptr) // must be set if enabled
	);

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
	map:  x+=1, y-=1, pos -= mapW-1
	rmap: x+=1, y+=1, pos += rmapW+1
	jump_east = map: jump_east(M1), (x+r,y)
	jump_north = rmap: jump_east(M0), (x,y-r)

	NW = jump_intercardinal_neg(M0=map,M1=rmap)
	map:  x-=1, y-=1, pos -= mapW+1
	rmap: x+=1, y-=1, pos -= rmapW-1
	jump_north = rmap: jump_east(M1), (x,y-r)
	jump_west = map: jump_west(M0), (x-r,y)

	SW = jump_intercardinal_neg(M0=rmap,M1=map)
	map:  x-=1, y+=1, pos += mapW-1
	rmap: x-=1, y-=1, pos -= rmapW+1
	jump_south = rmap: jump_west(M0), (x,y+r)
	jump_west = map: jump_west(M1), (x-r,y)
	*/

	IntercardinalWalker<D> walker; // class to walk
	// setup the walker members
	// 0 = map, 1 = rmap
	walker.map[0] = map_;
	walker.map[1] = rmap_;
	walker.map_width(map_.width());
	walker.rmap_width(rmap_.width());
	walker.node_at[0] = static_cast<uint32_t>(node);
	walker.node_at[1] = static_cast<uint32_t>(rnode);
	walker.goal[0]    = goal_.id;
	walker.goal[1]    = rgoal_.id;

	// pre-set cardinal results to none, in case no successors are found
	if constexpr(feature_store_cardinal())
	{
		result_node[0] = result_node[1] = jps_id::none();
	}

	// JPS, stop at the first intercardinal turning point
	if constexpr(!feature_prune_intercardinal())
	{
		uint32_t walk_count = 1;
		walker.first_row();
		while(true)
		{
			walker.next_row();      // walk_count adjusted at end of loop
			if(!walker.valid_row()) // no successors
				return {jps_id::none(), jps_rid::none(), 0};
			// check if intercardinal is passing over goal
			if(walker.node_at[0] == walker.goal[0]) [[unlikely]]
			{
				// reached goal
				intercardinal_jump_result result;
				result.node  = jps_id{walker.node_at[0]};
				result.rnode = jps_rid::none(); // walker.get_last_rrow();
				result.dist  = walk_count;
				return result;
			}
			//
			// handle hori/vert long jump
			//
			auto res = walker.long_jump();
			if(res.joint != 0)
			{ // at least one jump has a turning point
				intercardinal_jump_result result;
				if(!feature_store_cardinal())
				{
					// do not store cardinal results, just return the
					// intercardinal point
					result.node  = jps_id{walker.node_at[0]};
					result.rnode = jps_rid::none(); // walker.get_last_rrow();
					result.dist  = walk_count;
				}
				else
				{
					cost_t current_cost = walk_count * warthog::DBL_ROOT_TWO;
					// check hori/vert jump result and store their values
					if(res.dist[0] != 0)
					{
						result_node[0]
						    = walker.adj_hori(walker.node_at[0], res.dist[0]);
						result_cost[0]
						    = current_cost + res.dist[0] * warthog::DBL_ONE;
					}
					else { result_node[0] = jps_id::none(); }
					if(res.dist[1] != 0)
					{
						result_node[1]
						    = walker.adj_vert(walker.node_at[0], res.dist[1]);
						result_cost[1]
						    = current_cost + res.dist[1] * warthog::DBL_ONE;
					}
					else { result_node[1] = jps_id::none(); }
					// store next row location as we do not need to keep the
					// current intercardinal
					walker.next_row();
					result.node  = jps_id{walker.node_at[0]};
					result.rnode = jps_rid::none(); // walker.get_last_rrow();
					result.dist  = walker.valid_row() ? walk_count + 1 : 0;
				}
				// found jump point, return
				return result;
			}
			walk_count += 1;
		}
	}
	else
	{
		// prunes intercardinal, progress and add successors to count
		assert(result_size > 2);
		result_size
		    -= 1; // ensure there is always space for at least 2 results
		uint32_t walk_count   = 1;
		uint32_t result_count = 0;
		walker.first_row();
		// only continue if there is room to store results
		while(result_count < result_size)
		{
			walker.next_row();
			if(!walker.valid_row())
				return {jps_id::none(), jps_rid::none(), result_count};
			if(walker.node_at[0] == walker.goal[0]) [[unlikely]]
			{
				// reached goal
				result_count    += 1;
				*(result_node++) = jps_id{walker.node_at[0]};
				*(result_cost++) = walk_count * warthog::DBL_ROOT_TWO;
				break;
			}
			auto res            = walker.long_jump();
			cost_t current_cost = walk_count * warthog::DBL_ROOT_TWO;
			if(res.dist[0] != 0)
			{ // east/west
				result_count += 1;
				*(result_node++)
				    = walker.adj_hori(walker.node_at[0], res.dist[0]);
				*(result_cost++)
				    = current_cost + res.dist[0] * warthog::DBL_ONE;
			}
			if(res.dist[1] != 0)
			{ // north/south
				result_count += 1;
				// NORTH/SOUTH handles the correct sing, adjust for EAST/WEST
				// diff
				*(result_node++)
				    = walker.adj_vert(walker.node_at[0], res.dist[1]);
				*(result_cost++)
				    = current_cost + res.dist[1] * warthog::DBL_ONE;
			}
			walk_count += 1;
		}
		// not enough buffer, return result and start again
		return {
		    jps_id{walker.node_at[0]}, jps_rid{walker.node_at[1]},
		    result_count};
	}
}

}

#endif // JPS_JUMP_JUMP_POINT_ONLINE_H
