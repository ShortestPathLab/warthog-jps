#ifndef JPS_JUMP_JUMP_H
#define JPS_JUMP_JUMP_H

#include <jps/forward.h>

// Common types for jump

namespace jps::jump
{

using jump_distance = int16_t;

inline constexpr bool is_jump_point(jump_distance d) noexcept
{
	return d > 0;
}
inline constexpr bool is_deadend(jump_distance d) noexcept
{
	return d < 0;
}
inline constexpr bool is_blocked(jump_distance d) noexcept
{
	return d == 0;
}

inline constexpr direction_id get_hori_from_intercardinal(direction_id d) noexcept
{
	constexpr uint32_t map = ( static_cast<uint32_t>(EAST_ID) << 4 * static_cast<int>(NORTHEAST_ID) )
		| ( static_cast<uint32_t>(WEST_ID) << 4 * static_cast<int>(NORTHWEST_ID) )
		| ( static_cast<uint32_t>(EAST_ID) << 4 * static_cast<int>(SOUTHEAST_ID) )
		| ( static_cast<uint32_t>(WEST_ID) << 4 * static_cast<int>(SOUTHWEST_ID) );
	return static_cast<direction_id>(
		(map >> 4 * static_cast<int>(d)) & 0b1111
	);
}
inline constexpr direction_id get_vert_from_intercardinal(direction_id d) noexcept
{
	constexpr uint32_t map = ( static_cast<uint32_t>(NORTH_ID) << 4 * static_cast<int>(NORTHEAST_ID) )
		| ( static_cast<uint32_t>(NORTH_ID) << 4 * static_cast<int>(NORTHWEST_ID) )
		| ( static_cast<uint32_t>(SOUTH_ID) << 4 * static_cast<int>(SOUTHEAST_ID) )
		| ( static_cast<uint32_t>(SOUTH_ID) << 4 * static_cast<int>(SOUTHWEST_ID) );
	return static_cast<direction_id>(
		(map >> 4 * static_cast<int>(d)) & 0b1111
	);
}

struct intercardinal_jump_result
{
	jump_distance inter;
	jump_distance hori;
	jump_distance vert;
};

} // namespace jps::jump

#endif // JPS_JUMP_JUMP_H
