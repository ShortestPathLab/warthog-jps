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

/// @brief find first jump-point horizontal east (if East) or west on map.
///        Will land of target if Target is true.
/// @tparam East Jump east (true) or west (false)
/// @tparam Target if true, consider target as a jump-point as well.
/// @param map a map of the grid, only the width parameter is required
/// @param node the starting location
/// @param target the target location, only used if Target == true
/// @return positive distance to jump-point or target, otherwise negated distance to wall blocker
template<bool East, bool Target = true>
jump_distance
jump_point_online_hori(
    ::warthog::domain::gridmap::bittable map, uint32_t node, uint32_t target[[maybe_unused]] = std::numeric_limits<uint32_t>::max())
{
	assert(map.data() != nullptr);
	assert(map.size() == 0 || node < map.size());
	assert(map.get(grid_id(node)));
	assert(Target != (target == std::numeric_limits<uint32_t>::max()));
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
			uint32_t target_jump[[maybe_unused]] = Target ? (East ? target - node : node - target) : 0;
			// if blocked: pos + jump_count = first block
			//  otherwise: jump_count = trav cell after turn (jump point
			//  location)
			// check for target with target_jump (dist) <= jump_count, as if < than
			// target is reachable, if equal then trav pos is the target if
			// greater, than target is further or another row or behind (as
			// unsigned)
			assert(jump_count >= 0);
			// must be checked as unsigned for:
			// 1. target.is_none(): will not fit int32_t
			// 2. underflow means target is in opposite direction, desirable
			if(Target && (target_jump <= static_cast<uint32_t>(jump_count)))
			{
				// target reached
				jump_count = static_cast<jump_distance>(target_jump);
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

/// @brief find first jump-point horizontal east (if East) or west on map.
///        Will land of target if Target is true.
/// @tparam East Jump east (true) or west (false)
/// @tparam Target if true, consider target as a jump-point as well.
/// @param map a map of the grid, only the width parameter is required
/// @param node the starting location
/// @param target the target location, only used if Target == true
/// @return positive distance to jump-point or target, otherwise negated distance to wall blocker
template<bool East>
jump_distance
jump_point_online_hori_target(
    ::warthog::domain::gridmap::bittable map, uint32_t node, uint32_t target)
{
	assert(map.data() != nullptr);
	// read tiles from the grid:
	// - along the row of node_id
	// - from the row above node_id
	// - from the row below node_id
	// NB: the jump direction corresponds to moving from the
	// low bit of the tileset and towards the high bit (EAST)
	// or high bit to low bit (WEST)
	auto nei_slider
	    = ::warthog::domain::gridmap_slider::from_bittable(map, pad_id{node});
	if constexpr (!East) { // adjust to last byte for west
		nei_slider.adj_bytes(-7);
		nei_slider.width8_bits = 7 - nei_slider.width8_bits;
	}

	// order going east is stored as least significant bit to most significant
	// bit

	const uint32_t target_jump = East ? target - node : node - target;
	assert(nei_slider.width8_bits < 8);
	jump_distance jump_count = -static_cast<jump_distance>(nei_slider.width8_bits);
	// setup jump block, negate so trav is 0, mask out points before start
	// just mask
	uint64_t jump_block = East ? (~0ull << nei_slider.width8_bits)
						: (~0ull >> nei_slider.width8_bits);
	// negate and mask
	jump_block = ~nei_slider.get_block_64bit_le() & jump_block;

	while(true)
	{
		if(jump_block)
		{
			int stop_pos = East
			    ? std::countr_zero(jump_block)
			    : std::countl_zero(jump_block); // the location to stop at
			//  v is blocker location, prune unless target is present
			// 10111111
			// dead end takes president as jump point can't pass a blocker
			jump_count += static_cast<jump_distance>(stop_pos);
			// if blocked: pos + jump_count = first block
			//  otherwise: jump_count = trav cell after turn (jump point
			//  location)
			// check for target with target_jump (dist) <= jump_count, as if < than
			// target is reachable, if equal then trav pos is the target if
			// greater, than target is further or another row or behind (as
			// unsigned)
			assert(jump_count >= 0);
			if (static_cast<uint32_t>(jump_count) >= target_jump)
				return target_jump; // found target
			else
				return static_cast<jump_distance>(-(jump_count+1)); // no target
		}

		// no blockers, check for target
		jump_count += static_cast<jump_distance>(64);
		if (static_cast<uint32_t>(jump_count) >= target_jump)
			return target_jump; // found target
		nei_slider.adj_bytes(East ? 8 : -8);
		jump_block = ~nei_slider.get_block_64bit_le();
	}
}

struct BasicIntercardinalWalker
{
	using map_type = ::warthog::domain::gridmap::bitarray;
	/// @brief map and rmap (as bit array for small memory size)
	map_type map;
	/// @brief location of current node on map and rmap
	uint32_t node_at;
	/// @brief map and rmap value to adjust node_at for each row
	uint32_t adj_width;
	/// @brief row scan
	union
	{
		uint8_t row_[2]; ///< stores 3 bits at node_at[0]+-1, 0=prev,
		                 ///< 1=current; high order bits 3..7 are not zero'd
		uint16_t row_i_;
	};
	uint16_t row_mask_;

	template <direction_id D>
		requires InterCardinalId<D>
	void set_map(map_type map, uint32_t width) noexcept
	{
		this->map = map;
		this->adj_width = dir_id_adj(D, width);
		if constexpr (D == NORTHEAST_ID || D == SOUTHEAST_ID) {
			row_mask_ = std::endian::native == std::endian::little
			    ? 0b0000'0011'0000'0110
			    : 0b0000'0110'0000'0011;
		} else { // NORTHWEST_ID and SOUTHWEST_ID
			row_mask_ = std::endian::native == std::endian::little
			    ? 0b0000'0110'0000'0011
			    : 0b0000'0011'0000'0110;
		}
	}

	void
	next_index() noexcept
	{
		node_at += adj_width;
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

	/// @brief return get node_at-1..node_at+1 bits. CAUTION return
	/// bits 3..7 may not all be 0.
	uint8_t
	get_row() const noexcept
	{
		return static_cast<uint8_t>(
		    map.get_span<3>(pad_id{node_at - 1}));
	}
	/// @brief get node id for location node + dist * adj_width
	grid_id
	adj_id(uint32_t node, int32_t dist) const noexcept
	{
		return grid_id{static_cast<uint32_t>(node + static_cast<uint32_t>(dist) * adj_width)};
	}

	
	/// @brief the current locations row is a valid intercardinal move (i.e.
	/// 2x2 is free)
	bool
	valid_row() const noexcept
	{
		return (row_i_ & row_mask_) == row_mask_;
	}
	template <direction_id D>
		requires InterCardinalId<D>
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
};


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

		operator bool() const noexcept
		{
			return dist[0] > 0 || dist[1] > 0;
		}
	};
	using map_type = ::warthog::domain::gridmap::bitarray;
	/// @brief map and rmap (as bit array for small memory size)
	std::array<map_type, 2> map;
	/// @brief location of current node on map and rmap
	std::array<uint32_t, 2> node_at;
	/// @brief map and rmap value to adjust node_at for each row
	std::array<uint32_t, 2> adj_width;
	// /// @brief map and rmap target locations
	// uint32_t target[2];
	/// @brief row scan
	union
	{
		uint8_t row_[2]; ///< stores 3 bits at node_at[0]+-1, 0=prev,
		                 ///< 1=current; high order bits 3..7 are not zero'd
		uint16_t row_i_;
	};

	/// @brief convert map width to a map adj_width variable suited to
	/// intercardinal D2
	static constexpr uint32_t
	to_map_adj_width(uint32_t width) noexcept
	{
		return dir_id_adj(D, width);
	}
	/// @brief convert rmap width to a rmap adj_width variable suited to
	/// intercardinal D2
	static constexpr uint32_t
	to_rmap_adj_width(uint32_t width) noexcept
	{
		return dir_id_adj(dir_id_cw90(D), width);
	}

	/// @brief convert map adj_width to map width, reciprocal to
	/// to_map_adj_width
	static constexpr uint32_t
	from_map_adj_width(uint32_t adj_width) noexcept
	{
		return dir_id_adj_inv_intercardinal(D, adj_width);
	}
	/// @brief convert rmap adj_width to rmap width, reciprocal to
	/// to_rmap_adj_width
	static constexpr uint32_t
	from_rmap_adj_width(uint32_t adj_width) noexcept
	{
		return dir_id_adj_inv_intercardinal(dir_id_cw90(D), adj_width);
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
		jump_distance d = jump_point_online_hori<true, false>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node);
		// for LongJumpRes, must return 0 for deadend
		return d;
	}
	static jump_distance
	jump_east(map_type map, uint32_t width, uint32_t node, uint32_t target)
	{
		jump_distance d = jump_point_online_hori<true, true>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node, target);
		// for LongJumpRes, must return 0 for deadend
		return d;
	}
	static jump_distance
	jump_west(map_type map, uint32_t width, uint32_t node)
	{
		jump_distance d = jump_point_online_hori<false, false>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node);
		// for LongJumpRes, must return 0 for deadend
		return d;
	}
	static jump_distance
	jump_west(map_type map, uint32_t width, uint32_t node, uint32_t target)
	{
		jump_distance d = jump_point_online_hori<false, true>(
		    ::warthog::domain::gridmap::bittable(map, width, 0), node, target);
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
	jump_hori(grid_id target)
	{
		if constexpr(D == NORTHEAST_ID)
		{
			return jump_east(map[0], map_width(), node_at[0], static_cast<uint32_t>(target)); // east
		}
		else if constexpr(D == SOUTHEAST_ID)
		{
			return jump_east(map[0], map_width(), node_at[0], static_cast<uint32_t>(target)); // east
		}
		else if constexpr(D == SOUTHWEST_ID)
		{
			return jump_west(map[0], map_width(), node_at[0], static_cast<uint32_t>(target)); // west
		}
		else if constexpr(D == NORTHWEST_ID)
		{
			return jump_west(map[0], map_width(), node_at[0], static_cast<uint32_t>(target)); // west
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
	jump_vert(rgrid_id target)
	{
		if constexpr(D == NORTHEAST_ID)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1], static_cast<uint32_t>(target)); // north
		}
		else if constexpr(D == SOUTHEAST_ID)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1], static_cast<uint32_t>(target)); // south
		}
		else if constexpr(D == SOUTHWEST_ID)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1], static_cast<uint32_t>(target)); // south
		}
		else if constexpr(D == NORTHWEST_ID)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1], static_cast<uint32_t>(target)); // north
		}
	}

	void
	next_index() noexcept
	{
		node_at[0] += adj_width[0];
		node_at[1] += adj_width[1];
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
	using bittable = ::warthog::domain::gridmap::bittable;
	using rotate_grid = domain::gridmap_rotate_table_convs;

	jump_point_online() = default;
	jump_point_online(const rotate_grid& map)
	{
		set_map(map);
	}
	~jump_point_online() = default;

	void
	set_map(const rotate_grid& map);
	// void
	// set_target(jps_id target_id) noexcept;
	// void
	// set_target(point target_id) noexcept;

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

	/// @brief same as jump_cardinal_next(point) but is given the correct grid_id type
	/// @tparam D cardinal direction_id (NORTH_ID,SOUTH_ID,EAST_ID,WEST_ID)
	/// @param loc current location on the grid to start the jump from
	/// @return >0: jump point n steps away, <=0: blocker -n steps away
	template <direction_id D>
		requires CardinalId<D>
	jump_distance
	jump_cardinal_next(domain::grid_pair_id node_id);

    /// @brief returns all intercardinal jump points up to max_distance (default inf)
    ///        and max of results_size
    /// @tparam D intercardinal direction_id (NORTHEAST_ID,NORTHWEST_ID,SOUTHEAST_ID,SOUTHWEST_ID)
    /// @param loc current location on the grid to start the jump from
    /// @param result_size maximum number of results that can fit in result
    /// @param result pointer to results storage of at least size result_size
    /// @param max_distance the maximum intercardinal distance to scan to
    /// @return pair first: number of results returned. second: the end distance
    ///
    /// This function is designed to efficiently find all jump points intercardinally.
    /// The returned point is either on the final result, the max_distance location (loc + max_distance)
    /// or the point in-front of the blocker(deadend).
    /// This function is intended to be run multiple times by passing the return loc into the next row,
    /// until either reaching a deadend or some algorithmic specific stopping criteria.
	template <direction_id D>
		requires InterCardinalId<D>
	std::pair<uint16_t, jump_distance>
	jump_intercardinal_many(
	    domain::grid_pair_id node_id, intercardinal_jump_result* result, uint16_t result_size, jump_distance max_distance = std::numeric_limits<jump_distance>::max());
	
	/// @brief shoot ray to target point
	/// @param loc shoot from loc
	/// @param target shoot to target
	/// @return pair <intercardinal-distance, cardinal-distance>, if both >= 0 than target is visible,
	///         first<0 means intercardinal reaches blocker at -first distance (second will be -1)
	///         first>=0 second<0 means cardinal blocker at -second distance away
	///         second<0 mean target is blocked in general
	std::pair<jump_distance, jump_distance>
	jump_target(
	    domain::grid_pair_id node_id, point loc, point target);

	size_t
	mem()
	{
		return sizeof(this);
	}

protected:
	static int32_t
	jump_east(bittable map, uint32_t node)
	{
		return jump_point_online_hori<true, false>(map, node);
	}
	static int32_t
	jump_west(bittable map, uint32_t node)
	{
		return jump_point_online_hori<false, false>(map, node);
	}

protected:
	rotate_grid map_;
};

void
jump_point_online::set_map(const rotate_grid& orig)
{
	map_ = orig;
}

template <direction_id D>
	requires CardinalId<D>
jump_distance
jump_point_online::jump_cardinal_next(domain::grid_pair_id node_id)
{
	if constexpr (D == NORTH_ID || D == EAST_ID)
	{
		return jump_east(map_[domain::rgrid_index<D>], static_cast<uint32_t>(get_d<D>(node_id)));
	}
	else if constexpr (D == SOUTH_ID || D == WEST_ID)
	{
		return jump_west(map_[domain::rgrid_index<D>], static_cast<uint32_t>(get_d<D>(node_id)));
	} else {
		assert(false);
		return 0;
	}
}

template <direction_id D>
	requires InterCardinalId<D>
std::pair<uint16_t, jump_distance>
jump_point_online::jump_intercardinal_many(
	domain::grid_pair_id node_id, intercardinal_jump_result* result, uint16_t result_size, jump_distance max_distance)
{
	assert(result_size > 0);
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
	walker.map = {map_[0], map_[1]};
	walker.map_width(map_[0].width());
	walker.rmap_width(map_[1].width());
	walker.node_at[0] = static_cast<uint32_t>(get<grid_id>(node_id));
	walker.node_at[1] = static_cast<uint32_t>(get<rgrid_id>(node_id));
	assert(map_.map().get(grid_id(walker.node_at[0])) && map_.rmap().get(grid_id(walker.node_at[1])));

	// JPS, stop at the first intercardinal turning point
	uint16_t results_count = 0;
	jump_distance walk_count = 0;
	walker.first_row();
	while(walk_count < max_distance) // if equal, max distance is reached
	{
		walker.next_row();      // walk_count adjusted at end of loop
		if(!walker.valid_row())[[unlikely]] // no successors
			return {results_count, static_cast<jump_distance>(-walk_count)};
		walk_count += 1;
		//
		// handle hori/vert long jump
		//
		auto res = walker.long_jump();
		if(res)
		{ // at least one jump has a turning point
			assert(results_count < result_size);
			*(result++) = {walk_count, res.dist[0], res.dist[1]};
			if (++results_count == result_size)
				break;
		}
	}

	return {results_count, walk_count};
}


std::pair<jump_distance, jump_distance>
jump_point_online::jump_target(
	domain::grid_pair_id node_id, point loc, point target)
{
	// direction_id real_d = d != 255 ? static_cast<direction_id>(d) : warthog::grid::point_to_direction_id(loc, target);
	auto [xd, yd] = warthog::grid::point_signed_diff(loc, target);
	jump_distance inter_len;
	jump_distance cardinal_len;
	direction_id d;
	if (xd != 0 && yd != 0) {
		// diagonal
		BasicIntercardinalWalker walker;
		if (xd > 0) { // east
			if (yd > 0) { // south
				d = SOUTHEAST_ID;
				walker.set_map<SOUTHEAST_ID>(map_[0], map_[0].width());
			} else {
				d = NORTHEAST_ID;
				walker.set_map<NORTHEAST_ID>(map_[0], map_[0].width());
			}
		} else { // west
			if (yd > 0) { // south
				d = SOUTHWEST_ID;
				walker.set_map<SOUTHWEST_ID>(map_[0], map_[0].width());
			} else {
				d = NORTHWEST_ID;
				walker.set_map<NORTHWEST_ID>(map_[0], map_[0].width());
			}
		}
		walker.node_at = static_cast<uint32_t>(get<grid_id>(node_id));
		inter_len = static_cast<jump_distance>(std::min(std::abs(xd), std::abs(yd)));
		walker.first_row();
		for (jump_distance i = 0; i < inter_len; ++i) {
			walker.next_row();
			if (!walker.valid_row()) {
				// diagonal blocked before reaching turn, report
				return {static_cast<jump_distance>(-i), -1};
			}
		}
		// update loc
		spoint dia_unit = dir_unit_point(d);
		dia_unit.x *= inter_len; dia_unit.y *= inter_len;
		loc = loc + dia_unit;
		xd -= dia_unit.x; yd -= dia_unit.y;
	} else {
		// no diagonal
		inter_len = 0;
	}
	assert(xd == 0 || yd == 0);
	if (yd == 0) {
		// horizontal ray
		if (xd == 0) [[unlikely]] {
			// at target
			cardinal_len = 0;
		} else if (xd > 0) {
			// east
			cardinal_len = jump_point_online_hori_target<domain::rgrid_east<EAST_ID>>(
				map_[domain::rgrid_index<EAST_ID>],
				static_cast<uint32_t>(map_.point_to_id_d<EAST_ID>(loc)),
				static_cast<uint32_t>(map_.point_to_id_d<EAST_ID>(target)));
		} else {
			// west
			cardinal_len = jump_point_online_hori_target<domain::rgrid_east<WEST_ID>>(
				map_[domain::rgrid_index<WEST_ID>],
				static_cast<uint32_t>(map_.point_to_id_d<WEST_ID>(loc)),
				static_cast<uint32_t>(map_.point_to_id_d<WEST_ID>(target)));
		}
	} else if (yd > 0) {
		// south
		cardinal_len = jump_point_online_hori_target<domain::rgrid_east<SOUTH_ID>>(
			map_[domain::rgrid_index<SOUTH_ID>],
				static_cast<uint32_t>(map_.point_to_id_d<SOUTH_ID>(loc)),
				static_cast<uint32_t>(map_.point_to_id_d<SOUTH_ID>(target)));
	} else {
		// north
		cardinal_len = jump_point_online_hori_target<domain::rgrid_east<NORTH_ID>>(
			map_[domain::rgrid_index<NORTH_ID>],
				static_cast<uint32_t>(map_.point_to_id_d<NORTH_ID>(loc)),
				static_cast<uint32_t>(map_.point_to_id_d<NORTH_ID>(target)));
	}
	return {inter_len, cardinal_len};
}

}

#endif // JPS_JUMP_JUMP_POINT_ONLINE_H
