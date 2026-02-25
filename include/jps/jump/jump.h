#ifndef JPS_JUMP_JUMP_H
#define JPS_JUMP_JUMP_H

//
// jps/jump/jump.h
//
// Global jump header.
// Include types and basic utility functions.
//
// @author Ryan Hechenberger
// @created 2025-11-20
//

#include <jps/forward.h>

// Common types for jump

namespace jps::jump
{

using jump_distance = int16_t;

// functions for interpreting a jump point type from distance
inline constexpr bool
is_jump_point(jump_distance d) noexcept
{
	return d > 0;
}
inline constexpr bool
is_deadend(jump_distance d) noexcept
{
	return d < 0;
}
inline constexpr bool
is_blocked(jump_distance d) noexcept
{
	return d == 0;
}

// simple cast to unsigned jump distance, care as it is both widening and
// changing sign
inline constexpr uint32_t
to_unsigned_jump_distance(jump_distance d) noexcept
{
	return static_cast<uint32_t>(static_cast<int32_t>(d));
}

/// get the horizontal direction of an intercardinal
inline constexpr direction_id
get_hori_from_intercardinal(direction_id d) noexcept
{
	constexpr uint32_t map = (static_cast<uint32_t>(EAST_ID)
	                          << 4 * static_cast<int>(NORTHEAST_ID))
	    | (static_cast<uint32_t>(WEST_ID)
	       << 4 * static_cast<int>(NORTHWEST_ID))
	    | (static_cast<uint32_t>(EAST_ID)
	       << 4 * static_cast<int>(SOUTHEAST_ID))
	    | (static_cast<uint32_t>(WEST_ID)
	       << 4 * static_cast<int>(SOUTHWEST_ID));
	return static_cast<direction_id>(
	    (map >> 4 * static_cast<int>(d)) & 0b1111);
}
/// get the vertical direction of an intercardinal
inline constexpr direction_id
get_vert_from_intercardinal(direction_id d) noexcept
{
	constexpr uint32_t map = (static_cast<uint32_t>(NORTH_ID)
	                          << 4 * static_cast<int>(NORTHEAST_ID))
	    | (static_cast<uint32_t>(NORTH_ID)
	       << 4 * static_cast<int>(NORTHWEST_ID))
	    | (static_cast<uint32_t>(SOUTH_ID)
	       << 4 * static_cast<int>(SOUTHEAST_ID))
	    | (static_cast<uint32_t>(SOUTH_ID)
	       << 4 * static_cast<int>(SOUTHWEST_ID));
	return static_cast<direction_id>(
	    (map >> 4 * static_cast<int>(d)) & 0b1111);
}

/// @brief standard result type of an intercardianl jump.
///        Supports the inter-cardinal distance, as well as the jump-point
///        distance of the hori/vert from the inter (if exists)
struct intercardinal_jump_result
{
	jump_distance inter; ///< intercardinal distance (NE/NW/SE/SW)
	jump_distance hori;  ///< hori distance from inter (i.e.
	                     ///< get_hori_from_intercardinal(D))
	jump_distance vert;  ///< vert distance from inter (i.e.
	                     ///< get_vert_from_intercardinal(D))
};

} // namespace jps::jump

#endif // JPS_JUMP_JUMP_H
