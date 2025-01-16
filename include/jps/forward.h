#ifndef JPS_FORWARD_H
#define JPS_FORWARD_H

#include <vector>
#include <warthog/constants.h>
#include <warthog/forward.h>
#include <warthog/defines.h>

namespace jps
{

using warthog::pack_id;
using warthog::pad_id;
using jps_id = pad_id;

enum direction
{
	NONE = 0,
	NORTH = 1,
	SOUTH = 2,
	EAST = 4,
	WEST = 8,
	NORTHEAST = 16,
	NORTHWEST = 32,
	SOUTHEAST = 64,
	SOUTHWEST = 128,
	ALL = 255
};

using vec_jps_id = std::vector<jps_id>;
using vec_jps_cost = std::vector<warthog::cost_t>;

struct alignas(uint32_t) point
{
	uint16_t x;
	uint16_t y;
};

enum class JpsFeature : uint8_t
{
	DEFAULT = 0, // uses block-based jumping
	IMPROVE_PRUNING = 1 << 0,
#ifdef WARTHOG_INT128_ENABLED
	FORCE_INT128 = 1 << 1,
	TRY_INT128 = FORCE_INT128,
#else
	TRY_INT128 = 0
#endif
};
JpsFeature operator|(JpsFeature a, JpsFeature b) noexcept
{
	return static_cast<JpsFeature>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

} // namespace jps

#endif // JPS_SEARCH_FORWARD_H
