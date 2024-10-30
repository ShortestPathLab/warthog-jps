#ifndef JPS_SEARCH_FORWARD_H
#define JPS_SEARCH_FORWARD_H

#include "../forward.h"
#include <vector>

namespace jps::search
{

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
// constexpr uint32_t JPS_ID_MASK = (1 << 24) - 1;

using vec_jps_id = std::vector<jps_id>;

} // namespace jps::search

#endif // JPS_SEARCH_FORWARD_H
