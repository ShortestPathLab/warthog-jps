#ifndef JPS_SEARCH_JPS_H
#define JPS_SEARCH_JPS_H

// jps.h
//
// This file contains the namespace for common definitions
// required by the various classes that use Jump Points.
// Note that the operations defined here assume corner
// cutting is not allowed. This change requires some slight
// modification to the basic Jump Point Search method.
// For details see:
// [D Harabor and A Grastien, The JPS+ Pathfinding System, SoCS, 2012]
//
// @author: dharabor
// @created: 04/09/2012
//

#include <jps/forward.h>
#include <warthog/domain/gridmap.h>
#include <warthog/util/helpers.h>

#include <unordered_map>

namespace jps::search
{

// compute the direction of the last move on the canonical
// path from node @param n1_id to node @param n2_id.
//
// @return one of direction::{N, S, E, W}
inline direction
from_direction(grid_id n1_id_p, grid_id n2_id_p, uint32_t map_width_p)
{
	if(uint32_t{n1_id_p} == warthog::domain::GRID_ID_MAX) { return NONE; }

	int32_t x, y, x2, y2;
	warthog::util::index_to_xy(uint32_t{n1_id_p}, map_width_p, x, y);
	warthog::util::index_to_xy(uint32_t{n2_id_p}, map_width_p, x2, y2);
	int32_t dx = abs(x2 - x);
	int32_t dy = abs(y2 - y);

	// NB: this check could be skipped if we apply intermediate
	// diagonal pruning during search
	if(dx == dy)
	{
		if(x2 < x)
		{
			if(y2 < y) { return NORTHWEST; }
			return SOUTHWEST;
		}
		if(y2 < y) { return NORTHEAST; }
		return SOUTHEAST;
	}

	if(dx > dy)
	{
		if(x2 > x) { return EAST; }
		return WEST;
	}

	if(y2 > y) { return SOUTH; }
	return NORTH;
}

constexpr inline direction_id
from_direction(point p1, point p2)
{
	union {
		struct {
			int32_t x;
			int32_t y;
		} p;
		uint64_t xy;
	} c;
	c.p.x = static_cast<int32_t>(p2.x) - static_cast<int32_t>(p1.x);
	c.p.y = static_cast<int32_t>(p2.y) - static_cast<int32_t>(p1.y);

	if (c.p.x == 0) {
		return c.p.y >= 0 ? NORTH_ID : SOUTH_ID;
	} else if (c.p.y == 0) {
		return c.p.x >= 0 ? EAST_ID : WEST_ID;
	} else {
		// shift>> to mulitple of 4 (0b100)
		// (x < 0) = 0b0100
		// (y < 0) = 0b1000
		int shift = ( (static_cast<uint32_t>(c.p.x) >> (31-2)) & 0b0100 ) |
			( (static_cast<uint32_t>(c.p.y) >> (31-3)) & 0b1000 );
		assert((c.p.x > 0 && c.p.y > 0 && shift == 0)
			|| (c.p.x < 0 && c.p.y > 0 && shift == 4)
			|| (c.p.x > 0 && c.p.y < 0 && shift == 8)
			|| (c.p.x < 0 && c.p.y < 0 && shift == 12));
		return static_cast<direction_id>(
			static_cast<uint16_t>(
				(static_cast<uint16_t>(NORTHEAST_ID) << 0) |
				(static_cast<uint16_t>(NORTHWEST_ID) << 4) |
				(static_cast<uint16_t>(SOUTHEAST_ID) << 8) |
				(static_cast<uint16_t>(SOUTHWEST_ID) << 12)
			) >> shift
		);
	}
}

// compute the 4-connected canonical last move on the
// path from (px, py) to (x, y)
// inline direction
// from_direction_4c(int32_t px, int32_t py, int32_t x, int32_t y)
//{
//    int32_t delta_x  = (px - x);
//    int32_t delta_y  = (py - y);
//
//    direction d;
//    d = (delta_y < 0) ? NORTH : SOUTH;
//
//    if(delta_x)
//    {
//        d = (delta_x < 0) ? WEST : EAST;
//    }
//    return d;
//}

inline direction
from_direction_4c(grid_id n1_xy_id, grid_id n2_xy_id, uint32_t mapwidth)
{
	if(n1_xy_id.is_none()) { return NONE; }

	int32_t x1, y1, x2, y2;
	warthog::util::index_to_xy(uint32_t{n1_xy_id}, mapwidth, x1, y1);
	warthog::util::index_to_xy(uint32_t{n2_xy_id}, mapwidth, x2, y2);
	//    return warthog::jps::from_direction_4c(x1, y1, x2, y2);
	int32_t delta_x = (x1 - x2);
	int32_t delta_y = (y1 - y2);

	direction d;
	d = (delta_y < 0) ? SOUTH : NORTH;

	if(delta_x) { d = (delta_x < 0) ? EAST : WEST; }
	return d;
}

// Computes the set of "forced" directions in which to search for jump points
// from a given location (x, y).
// A neighbour is forced if it cannot be proven that there is at least one
// alternative optimal path that does not pass through the node (x, y).
uint32_t
compute_forced(direction d, uint32_t tiles);

// Computes the set of "natural" neighbours for a given location
// (x, y).
uint32_t
compute_natural(direction d, uint32_t tiles);

// Computes all successors (forced \union natural) of a node (x, y).
// This function is specialised for uniform cost grid maps.
//
// @param d: the direction of travel used to reach (x, y)
// @param tiles: the 3x3 square of cells having (x, y) at its centre.
//
// @return an integer representing the set of forced and natural directions.
// Each of the first 8 bits of the returned value, when set, correspond to a
// direction, as defined in direction
inline uint32_t
compute_successors(direction d, uint32_t tiles)
{
	return compute_forced(d, tiles) | compute_natural(d, tiles);
}

// Computes all the natural and forced directions of a node (x, y)
// This function is specialised for 4-connected uniform cost grid maps.
//
// @param d: the direction of travel used to reach (x, y)
// @param tiles: the 3x3 square of cells having (x, y) at its centre.
//
// @return an integer representing the set of forced and natural directions.
// The bits of the lowest byte, when set, each correspond to a specific
// direction, as defined in direction
inline uint32_t
compute_successors_4c(direction d, uint32_t tiles)
{
	uint32_t retval = 0;
	switch(d)
	{
	case NORTH:
	{
		// all natural, nothing forced
		retval = NORTH | EAST | WEST;
		break;
	}
	case SOUTH:
	{
		// all natural, nothing forced
		retval = SOUTH | EAST | WEST;
		break;
	}
	case EAST:
	{
		// natural
		retval = EAST;

		// forced
		uint32_t force_n = ((tiles & 3) == 2);
		retval          |= force_n; // force north

		uint32_t force_s = ((tiles & 196608) == 131072);
		retval          |= (force_s << 1); // force south
		break;
	}
	case WEST:
	{
		// natural
		retval = WEST;

		// forced
		uint32_t force_n = ((tiles & 6) == 2);
		retval          |= force_n; // force north

		uint32_t force_s = ((tiles & 393216) == 131072);
		retval          |= (force_s << 1); // force south
		break;
	}
	case NONE:
	{
		// all natural, nothing forced
		retval = NORTH | SOUTH | EAST | WEST;
		break;
	}
	default:
		break;
	}
	return retval;
}

#if 0
// creates a warthog::graph::xy_graph which contains only
// nodes that are jump points and edges which represent valid jumps,
// from one jump point to another.
//
// @param gm: the input grid
// @param id_map: a key/value set that maps the grid id of
// of each jump points to a corresponding id in the graph (optional)
//
//
// @return the jump point graph
warthog::graph::xy_graph*
create_jump_point_graph(warthog::domain::gridmap* gm);
#endif

// // given an input grid, create a new map where every (x, y) location
// // is labeled as a corner point or not.
// //
// // @param: gm; the input grid
// // @return the corner gridmap
// warthog::domain::gridmap*
// create_corner_map(warthog::domain::gridmap* gm);

} // namespace jps::search

#endif // JPS_SEARCH_JPS_H
