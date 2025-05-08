#include <jps/jump/online_jump_point_locator.h>
#include <jps/search/jps.h>
#include <warthog/domain/gridmap.h>

#include <cassert>
#include <climits>

namespace jps::jump
{

online_jump_point_locator::online_jump_point_locator(
    warthog::domain::gridmap* map)
    : map_(map) //, jumplimit_(UINT32_MAX)
{
	rmap_ = create_rmap();
}

online_jump_point_locator::~online_jump_point_locator()
{
	delete rmap_;
}

// create a copy of the grid map which is rotated by 90 degrees clockwise.
// this version will be used when jumping North or South.
warthog::domain::gridmap*
online_jump_point_locator::create_rmap()
{
	uint32_t maph  = map_->header_height();
	uint32_t mapw  = map_->header_width();
	uint32_t rmaph = mapw;
	uint32_t rmapw = maph;
	warthog::domain::gridmap* rmap
	    = new warthog::domain::gridmap(rmaph, rmapw);

	for(uint32_t x = 0; x < mapw; x++)
	{
		for(uint32_t y = 0; y < maph; y++)
		{
			uint32_t label
			    = map_->get_label(map_->to_padded_id_from_unpadded(x, y));
			uint32_t rx = ((rmapw - 1) - y);
			uint32_t ry = x;
			rmap->set_label(rmap->to_padded_id_from_unpadded(rx, ry), label);
		}
	}
	return rmap;
}

// Finds a jump point successor of node (x, y) in Direction d.
// Also given is the location of the goal node (goalx, goaly) for a particular
// search instance. If encountered, the goal node is always returned as a
// jump point successor.
//
// @return: the id of a jump point successor or jps_id::none() if no jp exists.
void
online_jump_point_locator::jump(
    direction d, jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost)
{
	switch(d)
	{
	case NORTH:
		jump_north(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case SOUTH:
		jump_south(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case EAST:
		jump_east(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case WEST:
		jump_west(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case NORTHEAST:
		jump_northeast(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case NORTHWEST:
		jump_northwest(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case SOUTHEAST:
		jump_southeast(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case SOUTHWEST:
		jump_southwest(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	default:
		break;
	}
}

void
online_jump_point_locator::jump_north(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost)
{
	node_id = this->map_id_to_rmap_id(node_id);
	goal_id = this->map_id_to_rmap_id(goal_id);
	jump_north_(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
	jumpnode_id = this->rmap_id_to_map_id(jumpnode_id);
}

void
online_jump_point_locator::jump_north_(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// east when we use a version of the map rotated 90 degrees.
	jump_east_(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
}

void
online_jump_point_locator::jump_south(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost)
{
	node_id = this->map_id_to_rmap_id(node_id);
	goal_id = this->map_id_to_rmap_id(goal_id);
	jump_south_(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
	jumpnode_id = this->rmap_id_to_map_id(jumpnode_id);
}

void
online_jump_point_locator::jump_south_(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// west when we use a version of the map rotated 90 degrees.
	jump_west_(node_id, goal_id, jumpnode_id, jumpcost, rmap_);
}

void
online_jump_point_locator::jump_east(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost)
{
	jump_east_(node_id, goal_id, jumpnode_id, jumpcost, map_);
}

void
online_jump_point_locator::jump_east_(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap)
{
	jumpnode_id = node_id;

	uint32_t neis[3] = {0, 0, 0};
	bool deadend     = false;

	jumpnode_id = node_id;
	while(true)
	{
		// read in tiles from 3 adjacent rows. the curent node
		// is in the low byte of the middle row
		mymap->get_neighbours_32bit(jumpnode_id, neis);

		// identity forced neighbours and deadend tiles.
		// forced neighbours are found in the top or bottom row. they
		// can be identified as a non-obstacle tile that follows
		// immediately  after an obstacle tile. A dead-end tile is
		// an obstacle found  on the middle row;
		uint32_t forced_bits  = (~neis[0] << 1) & neis[0];
		forced_bits          |= (~neis[2] << 1) & neis[2];
		uint32_t deadend_bits = ~neis[1];

		// stop if we found any forced or dead-end tiles
		int32_t stop_bits = (int32_t)(forced_bits | deadend_bits);
		if(stop_bits)
		{
			int32_t stop_pos = __builtin_ffs(stop_bits) - 1; // returns idx+1
			jumpnode_id.id  += (uint32_t)stop_pos;
			deadend          = deadend_bits & (1 << stop_pos);
			break;
		}

		// jump to the last position in the cache. we do not jump past the end
		// in case the last tile from the row above or below is an obstacle.
		// Such a tile, followed by a non-obstacle tile, would yield a forced
		// neighbour that we don't want to miss.
		jumpnode_id.id += 31;
	}

	uint32_t num_steps = jumpnode_id.id - node_id.id;
	uint32_t goal_dist = goal_id.id - node_id.id;
	if(num_steps > goal_dist)
	{
		jumpnode_id = goal_id;
		jumpcost    = goal_dist;
		return;
	}

	if(deadend)
	{
		// number of steps to reach the deadend tile is not
		// correct here since we just inverted neis[1] and then
		// looked for the first set bit. need -1 to fix it.
		num_steps  -= (1 && num_steps);
		jumpnode_id = jps_id::none();
	}
	jumpcost = num_steps;
}

// analogous to ::jump_east
void
online_jump_point_locator::jump_west(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost)
{
	jump_west_(node_id, goal_id, jumpnode_id, jumpcost, map_);
}

void
online_jump_point_locator::jump_west_(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap)
{
	bool deadend     = false;
	uint32_t neis[3] = {0, 0, 0};

	jumpnode_id = node_id;
	while(true)
	{
		// cache 32 tiles from three adjacent rows.
		// current tile is in the high byte of the middle row
		mymap->get_neighbours_upper_32bit(jumpnode_id, neis);

		// identify forced and dead-end nodes
		uint32_t forced_bits  = (~neis[0] >> 1) & neis[0];
		forced_bits          |= (~neis[2] >> 1) & neis[2];
		uint32_t deadend_bits = ~neis[1];

		// stop if we encounter any forced or deadend nodes
		uint32_t stop_bits = (forced_bits | deadend_bits);
		if(stop_bits)
		{
			uint32_t stop_pos = (uint32_t)__builtin_clz(stop_bits);
			jumpnode_id.id   -= stop_pos;
			deadend           = deadend_bits & (0x80000000 >> stop_pos);
			break;
		}
		// jump to the end of cache. jumping +32 involves checking
		// for forced neis between adjacent sets of contiguous tiles
		jumpnode_id.id -= 31;
	}

	uint32_t num_steps = node_id.id - jumpnode_id.id;
	uint32_t goal_dist = node_id.id - goal_id.id;
	if(num_steps > goal_dist)
	{
		jumpnode_id = goal_id;
		jumpcost    = goal_dist;
		return;
	}

	if(deadend)
	{
		// number of steps to reach the deadend tile is not
		// correct here since we just inverted neis[1] and then
		// counted leading zeroes. need -1 to fix it.
		num_steps  -= (1 && num_steps);
		jumpnode_id = jps_id::none();
	}
	jumpcost = num_steps;
}

void
online_jump_point_locator::jump_northeast(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost)
{
	uint32_t num_steps = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	jps_id next_id = node_id;
	uint32_t mapw  = map_->width();

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	uint32_t neis;
	map_->get_neighbours(next_id, (uint8_t*)&neis);
	if((neis & 1542) != 1542)
	{
		jumpnode_id = jps_id::none();
		jumpcost    = 0;
		return;
	}

	// jump a single step at a time (no corner cutting)
	jps_id rnext_id = map_id_to_rmap_id(next_id);
	jps_id rgoal_id = map_id_to_rmap_id(goal_id);
	uint32_t rmapw  = rmap_->width();
	while(true)
	{
		num_steps++;
		next_id  = jps_id(next_id.id - mapw + 1);
		rnext_id = jps_id(rnext_id.id + rmapw + 1);

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		jps_id jp_id1, jp_id2;
		warthog::cost_t cost1, cost2;
		jump_north_(rnext_id, rgoal_id, jp_id1, cost1, rmap_);
		if(!jp_id1.is_none()) { break; }
		jump_east_(next_id, goal_id, jp_id2, cost2, map_);
		if(!jp_id2.is_none()) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!((uint64_t)cost1 && (uint64_t)cost2))
		{
			next_id = jps_id::none();
			break;
		}
	}
	jumpnode_id = next_id;
	jumpcost    = num_steps * warthog::DBL_ROOT_TWO;
}

void
online_jump_point_locator::jump_northwest(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost)
{
	uint32_t num_steps = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	jps_id next_id = node_id;
	uint32_t mapw  = map_->width();

	// early termination (invalid first step)
	uint32_t neis;
	map_->get_neighbours(next_id, (uint8_t*)&neis);
	if((neis & 771) != 771)
	{
		jumpnode_id = jps_id::none();
		jumpcost    = 0;
		return;
	}

	// jump a single step at a time (no corner cutting)
	jps_id rnext_id = map_id_to_rmap_id(next_id);
	jps_id rgoal_id = map_id_to_rmap_id(goal_id);
	uint32_t rmapw  = rmap_->width();
	while(true)
	{
		num_steps++;
		next_id  = jps_id(next_id.id - mapw - 1);
		rnext_id = jps_id(rnext_id.id - (rmapw - 1));

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		jps_id jp_id1, jp_id2;
		warthog::cost_t cost1, cost2;
		jump_north_(rnext_id, rgoal_id, jp_id1, cost1, rmap_);
		if(!jp_id1.is_none()) { break; }
		jump_west_(next_id, goal_id, jp_id2, cost2, map_);
		if(!jp_id2.is_none()) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!((uint64_t)cost1 && (uint64_t)cost2))
		{
			next_id = jps_id::none();
			break;
		}
	}
	jumpnode_id = next_id;
	jumpcost    = num_steps * warthog::DBL_ROOT_TWO;
}

void
online_jump_point_locator::jump_southeast(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost)
{
	uint32_t num_steps = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	jps_id next_id = node_id;
	uint32_t mapw  = map_->width();

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	uint32_t neis;
	map_->get_neighbours(next_id, (uint8_t*)&neis);
	if((neis & 394752) != 394752)
	{
		jumpnode_id = jps_id::none();
		jumpcost    = 0;
		return;
	}

	// jump a single step at a time (no corner cutting)
	jps_id rnext_id = map_id_to_rmap_id(next_id);
	jps_id rgoal_id = map_id_to_rmap_id(goal_id);
	uint32_t rmapw  = rmap_->width();
	while(true)
	{
		num_steps++;
		next_id  = jps_id(next_id.id + mapw + 1);
		rnext_id = jps_id(rnext_id.id + rmapw - 1);

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		jps_id jp_id1, jp_id2;
		warthog::cost_t cost1, cost2;
		jump_south_(rnext_id, rgoal_id, jp_id1, cost1, rmap_);
		if(!jp_id1.is_none()) { break; }
		jump_east_(next_id, goal_id, jp_id2, cost2, map_);
		if(!jp_id2.is_none()) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!((uint64_t)cost1 && (uint64_t)cost2))
		{
			next_id = jps_id::none();
			break;
		}
	}
	jumpnode_id = next_id;
	jumpcost    = num_steps * warthog::DBL_ROOT_TWO;
}

void
online_jump_point_locator::jump_southwest(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    warthog::cost_t& jumpcost)
{
	uint32_t num_steps = 0;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	jps_id next_id = node_id;
	uint32_t mapw  = map_->width();

	// early termination (first step is invalid)
	map_->get_neighbours(next_id, (uint8_t*)&neis);
	if((neis & 197376) != 197376)
	{
		jumpnode_id = jps_id::none();
		jumpcost    = 0;
		return;
	}

	// jump a single step (no corner cutting)
	jps_id rnext_id = map_id_to_rmap_id(next_id);
	jps_id rgoal_id = map_id_to_rmap_id(goal_id);
	uint32_t rmapw  = rmap_->width();
	while(true)
	{
		num_steps++;
		next_id  = jps_id(next_id.id + mapw - 1);
		rnext_id = jps_id(rnext_id.id - (rmapw + 1));

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		jps_id jp_id1, jp_id2;
		warthog::cost_t cost1, cost2;
		jump_south_(rnext_id, rgoal_id, jp_id1, cost1, rmap_);
		if(!jp_id1.is_none()) { break; }
		jump_west_(next_id, goal_id, jp_id2, cost2, map_);
		if(!jp_id2.is_none()) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!((uint64_t)cost1 && (uint64_t)cost2))
		{
			next_id = jps_id::none();
			break;
		}
	}
	jumpnode_id = next_id;
	jumpcost    = num_steps * warthog::DBL_ROOT_TWO;
}

} // namespace jps::jump
