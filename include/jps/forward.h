#ifndef JPS_FORWARD_H
#define JPS_FORWARD_H

#include <vector>
#include <warthog/constants.h>
#include <warthog/forward.h>

namespace jps
{

using warthog::pack_id;
using warthog::pad_id;
using jps_id = warthog::pad32_id;

typedef enum
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
} direction;

// we sometimes store the id of a node in the lower 3 bytes of a word and
// use the upper byte to store something else (e.g. the parent direction)
[[deprecated]]
constexpr uint32_t JPS_ID_MASK
    = (1 << 24) - 1;

using vec_jps_id = std::vector<jps_id>;
using vec_jps_cost = std::vector<warthog::cost_t>;

} // namespace jps

#endif // JPS_SEARCH_FORWARD_H
