#ifndef JPS_JUMP_JUMP_POINT_ONLINE_H
#define JPS_JUMP_JUMP_POINT_ONLINE_H

// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011,
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//

#include <jps/forward.h>
#include "jump.h"
#include <array>
#include <memory>
#include <jps/domain/rotate_gridmap.h>

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
template<bool East, bool Goal = true>
jump_distance
jump_point_online_hori(
    ::warthog::domain::gridmap::bittable map, uint32_t node, uint32_t goal[[maybe_unused]] = std::numeric_limits<uint32_t>::max())
{
	assert(map.data() != nullptr);
	assert(Goal != (goal == std::numeric_limits<uint32_t>::max()));
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
	jump_distance jump_count = 0;

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
			jump_count += static_cast<jump_distance>(stop_pos)
			    - static_cast<jump_distance>(nei_slider.width8_bits);
			uint32_t goal_jump[[maybe_unused]] = Goal ? (East ? goal - node : node - goal) : 0;
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
			if(Goal && (goal_jump <= static_cast<uint32_t>(jump_count)))
			{
				// goal reached
				jump_count = static_cast<jump_distance>(goal_jump);
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
		jump_count += static_cast<jump_distance>(63 - nei_slider.width8_bits);
		nei_slider.adj_bytes(East ? 7 : -7);
		nei_slider.width8_bits = 7;
	}
}

} // namespace details


template<direction_id D>
	requires InterCardinalId<D>
struct IntercardinalWalker
{
	static_assert(
	    D == NORTHEAST_ID || D == NORTHWEST_ID || D == SOUTHEAST_ID || D == SOUTHWEST_ID,
	    "Must be intercardinal direction");
	union LongJumpRes
	{
		jump_distance dist[2]; /// distance hori/vert of D a jump is valid to
		uint32_t joint;
	};
	using map_type = ::warthog::domain::gridmap::bitarray;
	/// @brief map and rmap (as bit array for small memory size)
	map_type map[2];
	/// @brief location of current node on map and rmap
	uint32_t node_at[2];
	/// @brief map and rmap value to adjust node_at for each row
	int32_t adj_width[2];
	// /// @brief map and rmap goal locations
	// uint32_t goal[2];
	/// @brief row scan
	union
	{
		uint8_t row_[2]; ///< stores 3 bits at node_at[0]+-1, 0=prev,
		                 ///< 1=current; high order bits 3..7 are not zero'd
		uint16_t row_i_;
	};

	/// @brief convert map width to a map adj_width variable suited to
	/// intercardinal D2
	template<direction_id D2 = D>
		requires InterCardinalId<D>
	static constexpr int32_t
	to_map_adj_width(uint32_t width) noexcept
	{
		static_assert(
		    D2 == NORTHEAST_ID || D2 == NORTHWEST_ID || D2 == SOUTHEAST_ID
		        || D2 == SOUTHWEST_ID,
		    "Must be intercardinal direction");
		assert(width > 0);
		if constexpr(D2 == NORTHEAST_ID)
		{
			return -static_cast<int32_t>(width - 1); // - (mapW-1)
		}
		else if constexpr(D2 == SOUTHEAST_ID)
		{
			return static_cast<int32_t>(width + 1); // + (mapW+1)
		}
		else if constexpr(D2 == SOUTHWEST_ID)
		{
			return static_cast<int32_t>(width - 1); // + (mapW-1)
		}
		else
		{                                            // NORTHWEST_ID
			return -static_cast<int32_t>(width + 1); // - (mapW+1)
		}
	}
	/// @brief convert rmap width to a rmap adj_width variable suited to
	/// intercardinal D2
	template<direction_id D2 = D>
		requires InterCardinalId<D>
	static constexpr int32_t
	to_rmap_adj_width(uint32_t width) noexcept
	{
		return to_map_adj_width<dir_id_cw(D2)>(width);
	}

	/// @brief convert map adj_width to map width, reciprocal to
	/// to_map_adj_width
	template<direction_id D2 = D>
		requires InterCardinalId<D>
	static constexpr uint32_t
	from_map_adj_width(int32_t adj_width) noexcept
	{
		static_assert(
		    D2 == NORTHEAST_ID || D2 == NORTHWEST_ID || D2 == SOUTHEAST_ID
		        || D2 == SOUTHWEST_ID,
		    "Must be intercardinal direction");
		if constexpr(D2 == NORTHEAST_ID)
		{
			return static_cast<uint32_t>(-adj_width + 1);
		}
		else if constexpr(D2 == SOUTHEAST_ID)
		{
			return static_cast<uint32_t>(adj_width - 1);
		}
		else if constexpr(D2 == SOUTHWEST_ID)
		{
			return static_cast<uint32_t>(adj_width + 1);
		}
		else
		{ // NORTHWEST_ID
			return static_cast<uint32_t>(-adj_width - 1);
		}
	}
	/// @brief convert rmap adj_width to rmap width, reciprocal to
	/// to_rmap_adj_width
	template<direction_id D2 = D>
		requires InterCardinalId<D>
	static constexpr uint32_t
	from_rmap_adj_width(int32_t adj_width) noexcept
	{
		return from_map_adj_width<dir_id_cw(D2)>(adj_width);
	}

	/// @brief set map width
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

	static jump_distance
	jump_east(map_type map, uint32_t width, uint32_t node)
	{
		jump_distance d = details::jump_point_online_hori<true, false>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node);
		// for LongJumpRes, must return 0 for deadend
		return d;
	}
	static jump_distance
	jump_east(map_type map, uint32_t width, uint32_t node, uint32_t goal)
	{
		jump_distance d = details::jump_point_online_hori<true, true>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node, goal);
		// for LongJumpRes, must return 0 for deadend
		return d;
	}
	static jump_distance
	jump_west(map_type map, uint32_t width, uint32_t node)
	{
		jump_distance d = details::jump_point_online_hori<false, false>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node);
		// for LongJumpRes, must return 0 for deadend
		return d;
	}
	static jump_distance
	jump_west(map_type map, uint32_t width, uint32_t node, uint32_t goal)
	{
		jump_distance d = details::jump_point_online_hori<false, true>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node, goal);
		// for LongJumpRes, must return 0 for deadend
		return d;
	}
	jump_distance
	jump_hori()
	{
		if constexpr(D == NORTHEAST_ID)
		{
			return jump_east(map[0], map_width(), node_at[0]); // east
		}
		else if constexpr(D == SOUTHEAST_ID)
		{
			return jump_east(map[0], map_width(), node_at[0]); // east
		}
		else if constexpr(D == SOUTHWEST_ID)
		{
			return jump_west(map[0], map_width(), node_at[0]); // west
		}
		else if constexpr(D == NORTHWEST_ID)
		{
			return jump_west(map[0], map_width(), node_at[0]); // west
		}
	}
	jump_distance
	jump_hori(grid_id goal)
	{
		if constexpr(D == NORTHEAST_ID)
		{
			return jump_east(map[0], map_width(), node_at[0], static_cast<uint32_t>(goal)); // east
		}
		else if constexpr(D == SOUTHEAST_ID)
		{
			return jump_east(map[0], map_width(), node_at[0], static_cast<uint32_t>(goal)); // east
		}
		else if constexpr(D == SOUTHWEST_ID)
		{
			return jump_west(map[0], map_width(), node_at[0], static_cast<uint32_t>(goal)); // west
		}
		else if constexpr(D == NORTHWEST_ID)
		{
			return jump_west(map[0], map_width(), node_at[0], static_cast<uint32_t>(goal)); // west
		}
	}
	jump_distance
	jump_vert()
	{
		if constexpr(D == NORTHEAST_ID)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1]); // north
		}
		else if constexpr(D == SOUTHEAST_ID)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1]); // south
		}
		else if constexpr(D == SOUTHWEST_ID)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1]); // south
		}
		else if constexpr(D == NORTHWEST_ID)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1]); // north
		}
	}
	jump_distance
	jump_vert(rgrid_id goal)
	{
		if constexpr(D == NORTHEAST_ID)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1], static_cast<uint32_t>(goal)); // north
		}
		else if constexpr(D == SOUTHEAST_ID)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1], static_cast<uint32_t>(goal)); // south
		}
		else if constexpr(D == SOUTHWEST_ID)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1], static_cast<uint32_t>(goal)); // south
		}
		else if constexpr(D == NORTHWEST_ID)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1], static_cast<uint32_t>(goal)); // north
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

	/// @brief return get node_at[0]-1..node_at[0]+1 bits. CAUTION return
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
	grid_id
	adj_hori(uint32_t node, uint32_t dist) const noexcept
	{
		if constexpr(D == NORTHEAST_ID || D == SOUTHEAST_ID)
		{
			return grid_id{node + dist};
		}
		else { return grid_id{node - dist}; }
	}
	/// @brief get node id for location node + dist(NORTH/SOUTH of D)
	rgrid_id
	adj_vert(uint32_t node, uint32_t dist) const noexcept
	{
		if constexpr(D == NORTHEAST_ID || D == SOUTHEAST_ID)
		{
			return rgrid_id{node + (adj_width[0] - 1) * dist};
		}
		else { return rgrid_id{node + (adj_width[0] + 1) * dist}; }
	}
	/// @brief the current locations row is a valid intercardinal move (i.e.
	/// 2x2 is free)
	bool
	valid_row() const noexcept
	{
		// east | west differernce
		// north/south does not make a difference
		if constexpr(D == NORTHEAST_ID || D == SOUTHEAST_ID)
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

class jump_point_online
{
public:
	using rgridmap = domain::rotate_gridmap;
	using bittable = ::warthog::domain::gridmap::bittable;
	using rotate_grid = domain::gridmap_rotate_table_convs;

	jump_point_online();
	jump_point_online(const rgridmap& map)
	{
		set_map(map);
	}
	~jump_point_online() = default;

	void
	set_map(const rgridmap& map);
	// void
	// set_goal(jps_id goal_id) noexcept;
	// void
	// set_goal(point goal_id) noexcept;

	// /// @brief avoid modifying these grids accidentally
	// bittable
	// map() const noexcept
	// {
	// 	return map_;
	// }
	// /// @brief care should be taken to avoid modifying these grids
	// bittable
	// rmap() noexcept
	// {
	// 	return rmap_;
	// }

	/// @brief returns cardinal jump distance to next jump point
	/// @tparam D cardinal direction_id (NORTH_ID,SOUTH_ID,EAST_ID,WEST_ID)
	/// @param loc current location on the grid to start the jump from
	/// @return >0: jump point n steps away, <=0: blocker -n steps away
	template <direction_id D>
		requires CardinalId<D>
	jump_distance
	jump_cardinal_next(point loc);
	/// @brief same as jump_cardinal_next(point) but is given the correct grid_id type
	/// @tparam D cardinal direction_id (NORTH_ID,SOUTH_ID,EAST_ID,WEST_ID)
	/// @param loc current location on the grid to start the jump from
	/// @return >0: jump point n steps away, <=0: blocker -n steps away
	template <direction_id D>
		requires CardinalId<D>
	jump_distance
	jump_cardinal_next(domain::direction_grid_id_t<D> node_id);

	/// @brief returns the next jump point intercardinal from loc,
	///        where one of hori or vertical jump_cardinal_next has a jump point
	/// @tparam D intercardinal direction_id (NORTHEAST_ID,NORTHWEST_ID,SOUTHEAST_ID,SOUTHWEST_ID)
	/// @param loc current location on the grid to start the jump from
	/// @return >0: jump point n steps away, <=0: blocker -n steps away
	template <direction_id D>
		requires InterCardinalId<D>
	intercardinal_jump_result
	jump_intercardinal_next(
	    point loc);

    /// @brief returns all intercardinal jump points up to max_distance (default inf)
    ///        and max of results_size
    /// @tparam D intercardinal direction_id (NORTHEAST_ID,NORTHWEST_ID,SOUTHEAST_ID,SOUTHWEST_ID)
    /// @param loc current location on the grid to start the jump from
    /// @param result_size maximum number of results that can fit in result
    /// @param result pointer to results storage of at least size result_size
    /// @param max_distance the maximum intercardinal distance to scan to
    /// @return pair first: number of results returned. second: the end point
    ///
    /// This function is designed to efficiently find all jump points intercardinally.
    /// The returned point is either on the final result, the max_distance location (loc + max_distance)
    /// or the point in-front of the blocker(deadend).
    /// This function is intended to be run multiple times by passing the return loc into the next row,
    /// until either reaching a deadend or some algorithmic specific stopping criteria.
	template <direction_id D>
		requires InterCardinalId<D>
	std::pair<int, point>
	jump_intercardinal_many(
	    point loc, int result_size, intercardinal_jump_result* result, jump_distance max_distance = std::numeric_limits<jump_distance>::max());

	size_t
	mem()
	{
		return sizeof(this);
	}

protected:
	static int32_t
	jump_east(bittable map, uint32_t node)
	{
		return details::jump_point_online_hori<true>(map, node);
	}
	static int32_t
	jump_west(bittable map, uint32_t node)
	{
		return details::jump_point_online_hori<false>(map, node);
	}

protected:
	rotate_grid map_;
};

void
jump_point_online::set_map(const rgridmap& orig)
{
	map_ = orig;
}

template <direction_id D>
	requires CardinalId<D>
jump_distance
jump_point_online::jump_cardinal_next(point loc)
{
	return jump_cardinal_next<D>(map_.point_to_id_d<D>(loc));
}

template <direction_id D>
	requires CardinalId<D>
jump_distance
jump_point_online::jump_cardinal_next(domain::direction_grid_id_t<D> node_id)
{
	if constexpr (D == NORTH_ID || D == EAST_ID)
	{
		return jump_east(map_[domain::direction_grid_index<D>], node_id);
	}
	else if constexpr (D == SOUTH_ID || D == WEST_ID)
	{
		return jump_west(map_[domain::direction_grid_index<D>], node_id);
	} else {
		assert(false);
		return 0;
	}
}

template <direction_id D>
	requires InterCardinalId<D>
std::pair<int, point>
jump_point_online::jump_intercardinal_many(
	point loc, int result_size, intercardinal_jump_result* result, jump_distance max_distance)
{
	assert(!node.is_none() && !rnode.is_none());

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
	walker.map = map_;
	walker.map[1] = map_[1];
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
