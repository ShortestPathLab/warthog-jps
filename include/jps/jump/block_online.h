#ifndef JPS_JUMP_BLOCK_ONLINE_H
#define JPS_JUMP_BLOCK_ONLINE_H

//
// jps/jump/block_online.h
//
// Block-based jump-point location fuctions, used in jump_point_online.
// Handles low-level block jumps, scanning up to 56 cells (bits) at a time.
//
// The jump_point_online_hori function locates jump-points accross a maps hori;
// providing the rotated map gives a vert version.
//
// IntercardinalWalker<D> is an intercardinal (NE/NW/SE/SW) jump-point locator
// utility structure.  Gives low-level operations so users easily create their
// own fast intercardinal expanders.
// The BasicIntercardinalWalker is a non-templated version of
// IntercardinalWalker.
//
// @author dharabor & Ryan Hechenberger
// @created 2025-11-20
//

#include "jump.h"
#include <array>
#include <jps/domain/rotate_gridmap.h>
#include <jps/forward.h>
#include <memory>
#include <warthog/util/template.h>

namespace jps::jump
{

/// @brief find first jump-point horizontal east (if East) or west on map.
///        Will land of target if Target is true.
/// @tparam East Jump east (true) or west (false)
/// @tparam Target if true, consider target as a jump-point as well.
/// @param map a map of the grid, only the width parameter is required
/// @param node the starting location
/// @param target the target location, only used if Target == true
/// @return positive distance to jump-point or target, otherwise negated
/// distance to wall blocker
template<bool East, bool Target = true>
jump_distance
jump_point_online_hori(
    ::warthog::domain::gridmap::bittable map, uint32_t node,
    uint32_t target [[maybe_unused]] = std::numeric_limits<uint32_t>::max())
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
	if constexpr(!East)
	{
		nei_slider.adj_bytes(-7); // west is opposite side
		nei_slider.width8_bits = 7u - nei_slider.width8_bits;
	}
	assert(nei_slider.width8_bits < 8);
	// width8_bits is how many bits in on word current node is at, from lsb
	// EAST and from msb WEST
	// 7 - width8_bits == 63 - (width8_bits + 7 * 8)

	// order going east is stored as least significant bit to most significant
	// bit

	// first loop gets neis as 8 <= width8_bits < 16
	// other loops will have width8_bits = 7
	std::array<uint64_t, 3> neis = nei_slider.get_neighbours_64bit_le();
	// the rest will only jump 7
	// shift above and below 2 points east
	// mask out to trav(1) before node location
	assert(nei_slider.width8_bits < 8);
	uint64_t tmp = East ? ~(~0ull << nei_slider.width8_bits)
	                    : ~(~0ull >> nei_slider.width8_bits);
	neis[0]     |= tmp;
	neis[1]     |= tmp;
	neis[2]     |= tmp;

	jump_distance jump_count
	    = 7u - static_cast<jump_distance>(nei_slider.width8_bits);

	while(true)
	{
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
			// first jump may not skip 7, this is adjusted for on init
			jump_count += static_cast<jump_distance>(stop_pos) - 7;
			uint32_t target_jump [[maybe_unused]]
			= Target ? (East ? target - node : node - target) : 0;
			// if blocked: pos + jump_count = first block
			//  otherwise: jump_count = trav cell after turn (jump point
			//  location)
			// check for target with target_jump (dist) <= jump_count, as if <
			// then target is reachable, if equal then trav pos is the target
			// if greater, then target is further or another row or behind (as
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
		jump_count += static_cast<jump_distance>(63 - 7);
		// nei_slider.width8_bits = 7;
		nei_slider.adj_bytes(East ? 7 : -7);

		// get next neis at end of loop
		neis = nei_slider.get_neighbours_64bit_le();
		// mask out to trav(1) is not nessesary on loop, as we know it is clear
		// constexpr uint64_t neis_mask = East ? ~(~0ull << 7) : ~(~0ull >> 7);
		// neis[0] |= neis_mask;
		// neis[1] |= neis_mask;
		// neis[2] |= neis_mask;
	}
}

/// @brief find first jump-point horizontal east (if East) or west on map.
///        Will land of target if Target is true.
/// @tparam East Jump east (true) or west (false)
/// @tparam Target if true, consider target as a jump-point as well.
/// @param map a map of the grid, only the width parameter is required
/// @param node the starting location
/// @param target the target location, only used if Target == true
/// @return positive distance to jump-point or target, otherwise negated
/// distance to wall blocker
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
	if constexpr(!East)
	{ // adjust to last byte for west
		nei_slider.adj_bytes(-7);
		nei_slider.width8_bits = 7u - nei_slider.width8_bits;
	}

	// order going east is stored as least significant bit to most significant
	// bit

	const uint32_t target_jump = East ? target - node : node - target;
	assert(nei_slider.width8_bits < 8);
	jump_distance jump_count
	    = -static_cast<jump_distance>(nei_slider.width8_bits);
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
			// check for target with target_jump (dist) <= jump_count, as if <
			// then target is reachable, if equal then trav pos is the target
			// if greater, then target is further or another row or behind (as
			// unsigned)
			assert(jump_count >= 0);
			if(static_cast<uint32_t>(jump_count) >= target_jump)
				return target_jump; // found target
			else
				return static_cast<jump_distance>(
				    -(jump_count - 1)); // no target
		}

		// no blockers, check for target
		jump_count += static_cast<jump_distance>(64);
		if(static_cast<uint32_t>(jump_count) >= target_jump)
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

	template<direction_id D>
	    requires InterCardinalId<D>
	void
	set_map(map_type map, uint32_t width) noexcept
	{
		this->map       = map;
		this->adj_width = dir_id_adj(D, width);
		if constexpr(D == NORTHEAST_ID || D == SOUTHEAST_ID)
		{
			row_mask_ = std::endian::native == std::endian::little
			    ? 0b0000'0011'0000'0110
			    : 0b0000'0110'0000'0011;
		}
		else
		{ // NORTHWEST_ID and SOUTHWEST_ID
			row_mask_ = std::endian::native == std::endian::little
			    ? 0b0000'0110'0000'0011
			    : 0b0000'0011'0000'0110;
		}
	}

	void
	set_map(direction_id d, map_type map, uint32_t width) noexcept
	{
		assert(is_intercardinal_id(d));
		warthog::util::choose_integer_sequence<std::integer_sequence<
		    direction_id, NORTHEAST_ID, NORTHWEST_ID, SOUTHEAST_ID,
		    SOUTHWEST_ID>>(d, [&, map, width](auto dv) {
			set_map<decltype(dv)::value>(map, width);
		});
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
		return static_cast<uint8_t>(map.get_span<3>(pad_id{node_at - 1}));
	}
	/// @brief get node id for location node + dist * adj_width
	grid_id
	adj_id(uint32_t node, int32_t dist) const noexcept
	{
		return grid_id{static_cast<uint32_t>(
		    node + static_cast<uint32_t>(dist) * adj_width)};
	}

	/// @brief the current locations row is a valid intercardinal move (i.e.
	/// 2x2 is free)
	bool
	valid_row() const noexcept
	{
		return (row_i_ & row_mask_) == row_mask_;
	}
	template<direction_id D>
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
	    D == NORTHEAST_ID || D == NORTHWEST_ID || D == SOUTHEAST_ID
	        || D == SOUTHWEST_ID,
	    "Must be intercardinal direction");
	union LongJumpRes
	{
		jump_distance dist[2]; /// distance hori/vert of D a jump is valid to

		operator bool() const noexcept { return dist[0] > 0 || dist[1] > 0; }
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
			return jump_east(
			    map[0], map_width(), node_at[0],
			    static_cast<uint32_t>(target)); // east
		}
		else if constexpr(D == SOUTHEAST_ID)
		{
			return jump_east(
			    map[0], map_width(), node_at[0],
			    static_cast<uint32_t>(target)); // east
		}
		else if constexpr(D == SOUTHWEST_ID)
		{
			return jump_west(
			    map[0], map_width(), node_at[0],
			    static_cast<uint32_t>(target)); // west
		}
		else if constexpr(D == NORTHWEST_ID)
		{
			return jump_west(
			    map[0], map_width(), node_at[0],
			    static_cast<uint32_t>(target)); // west
		}
	}
	jump_distance
	jump_vert()
	{
		if constexpr(D == NORTHEAST_ID)
		{
			return jump_east(map[1], rmap_width(), node_at[1]); // north
		}
		else if constexpr(D == SOUTHEAST_ID)
		{
			return jump_west(map[1], rmap_width(), node_at[1]); // south
		}
		else if constexpr(D == SOUTHWEST_ID)
		{
			return jump_west(map[1], rmap_width(), node_at[1]); // south
		}
		else if constexpr(D == NORTHWEST_ID)
		{
			return jump_east(map[1], rmap_width(), node_at[1]); // north
		}
	}
	jump_distance
	jump_vert(rgrid_id target)
	{
		if constexpr(D == NORTHEAST_ID)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1],
			    static_cast<uint32_t>(target)); // north
		}
		else if constexpr(D == SOUTHEAST_ID)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1],
			    static_cast<uint32_t>(target)); // south
		}
		else if constexpr(D == SOUTHWEST_ID)
		{
			return jump_west(
			    map[1], rmap_width(), node_at[1],
			    static_cast<uint32_t>(target)); // south
		}
		else if constexpr(D == NORTHWEST_ID)
		{
			return jump_east(
			    map[1], rmap_width(), node_at[1],
			    static_cast<uint32_t>(target)); // north
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

} // namespace jps::jump

#endif // JPS_JUMP_BLOCK_ONLINE_H
