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

#include "forward.h"
#include <warthog/domain/gridmap.h>
#include <warthog/util/helpers.h>

#include <stdint.h>
#include <unordered_map>

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
const uint32_t JPS_ID_MASK = (1 << 24) - 1;

// compute the direction of the last move on the canonical
// path from node @param n1_id to node @param n2_id.
//
// @return one of direction::{N, S, E, W}
direction inline from_direction(
    uint32_t n1_id_p, uint32_t n2_id_p, uint32_t map_width_p)
{
	if(n1_id_p == warthog::GRID_ID_MAX) { return NONE; }

	int32_t x, y, x2, y2;
	warthog::helpers::index_to_xy(n1_id_p, map_width_p, x, y);
	warthog::helpers::index_to_xy(n2_id_p, map_width_p, x2, y2);
	int32_t dx = abs(x2 - x);
	int32_t dy = abs(y2 - y);

	// NB: this check could be skipped if we apply intermediate
	// diagonal pruning during search
	if(dx == dy)
	{
		if(x2 < x)
		{
			if(y2 < y) { return warthog::jps::NORTHWEST; }
			return warthog::jps::SOUTHWEST;
		}
		if(y2 < y) { return warthog::jps::NORTHEAST; }
		return warthog::jps::SOUTHEAST;
	}

	if(dx > dy)
	{
		if(x2 > x) { return warthog::jps::EAST; }
		return warthog::jps::WEST;
	}

	if(y2 > y) { return warthog::jps::SOUTH; }
	return warthog::jps::NORTH;
}

// compute the 4-connected canonical last move on the
// path from (px, py) to (x, y)
// inline warthog::jps::direction
// from_direction_4c(int32_t px, int32_t py, int32_t x, int32_t y)
//{
//    int32_t delta_x  = (px - x);
//    int32_t delta_y  = (py - y);
//
//    warthog::jps::direction d;
//    d = (delta_y < 0) ? warthog::jps::NORTH : warthog::jps::SOUTH;
//
//    if(delta_x)
//    {
//        d = (delta_x < 0) ? warthog::jps::WEST : warthog::jps::EAST;
//    }
//    return d;
//}

inline warthog::jps::direction
from_direction_4c(uint32_t n1_xy_id, uint32_t n2_xy_id, uint32_t mapwidth)
{
	if(n1_xy_id == warthog::GRID_ID_MAX) { return warthog::jps::NONE; }

	int32_t x1, y1, x2, y2;
	warthog::helpers::index_to_xy(n1_xy_id, mapwidth, x1, y1);
	warthog::helpers::index_to_xy(n2_xy_id, mapwidth, x2, y2);
	//    return warthog::jps::from_direction_4c(x1, y1, x2, y2);
	int32_t delta_x = (x1 - x2);
	int32_t delta_y = (y1 - y2);

	warthog::jps::direction d;
	d = (delta_y < 0) ? warthog::jps::SOUTH : warthog::jps::NORTH;

	if(delta_x)
	{
		d = (delta_x < 0) ? warthog::jps::EAST : warthog::jps::WEST;
	}
	return d;
}

// Computes the set of "forced" directions in which to search for jump points
// from a given location (x, y).
// A neighbour is forced if it cannot be proven that there is at least one
// alternative optimal path that does not pass through the node (x, y).
uint32_t
compute_forced(warthog::jps::direction d, uint32_t tiles);

// Computes the set of "natural" neighbours for a given location
// (x, y).
uint32_t
compute_natural(warthog::jps::direction d, uint32_t tiles);

// Computes all successors (forced \union natural) of a node (x, y).
// This function is specialised for uniform cost grid maps.
//
// @param d: the direction of travel used to reach (x, y)
// @param tiles: the 3x3 square of cells having (x, y) at its centre.
//
// @return an integer representing the set of forced and natural directions.
// Each of the first 8 bits of the returned value, when set, correspond to a
// direction, as defined in warthog::jps::direction
inline uint32_t
compute_successors(warthog::jps::direction d, uint32_t tiles)
{
	return warthog::jps::compute_forced(d, tiles)
	    | warthog::jps::compute_natural(d, tiles);
}

// Computes all the natural and forced directions of a node (x, y)
// This function is specialised for 4-connected uniform cost grid maps.
//
// @param d: the direction of travel used to reach (x, y)
// @param tiles: the 3x3 square of cells having (x, y) at its centre.
//
// @return an integer representing the set of forced and natural directions.
// The bits of the lowest byte, when set, each correspond to a specific
// direction, as defined in warthog::jps::direction
inline uint32_t
compute_successors_4c(warthog::jps::direction d, uint32_t tiles)
{
	uint32_t retval = 0;
	switch(d)
	{
	case warthog::jps::NORTH:
	{
		// all natural, nothing forced
		retval = warthog::jps::NORTH | warthog::jps::EAST | warthog::jps::WEST;
		break;
	}
	case warthog::jps::SOUTH:
	{
		// all natural, nothing forced
		retval = warthog::jps::SOUTH | warthog::jps::EAST | warthog::jps::WEST;
		break;
	}
	case warthog::jps::EAST:
	{
		// natural
		retval = warthog::jps::EAST;

		// forced
		uint32_t force_n = ((tiles & 3) == 2);
		retval |= force_n; // force north

		uint32_t force_s = ((tiles & 196608) == 131072);
		retval |= (force_s << 1); // force south
		break;
	}
	case warthog::jps::WEST:
	{
		// natural
		retval = warthog::jps::WEST;

		// forced
		uint32_t force_n = ((tiles & 6) == 2);
		retval |= force_n; // force north

		uint32_t force_s = ((tiles & 393216) == 131072);
		retval |= (force_s << 1); // force south
		break;
	}
	case warthog::jps::NONE:
	{
		// all natural, nothing forced
		retval = warthog::jps::NORTH | warthog::jps::SOUTH | warthog::jps::EAST
		    | warthog::jps::WEST;
		break;
	}
	default:
		break;
	}
	return retval;
}

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

// given an input grid, create a new map where every (x, y) location
// is labeled as a corner point or not.
//
// @param: gm; the input grid
// @return the corner gridmap
warthog::domain::gridmap*
create_corner_map(warthog::domain::gridmap* gm);

}

#endif // JPS_SEARCH_JPS_H
