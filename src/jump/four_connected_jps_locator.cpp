#include <jps/jump/four_connected_jps_locator.h>
#include <jps/search/jps.h>

#include <cassert>
#include <climits>

namespace jps::jump
{

four_connected_jps_locator::four_connected_jps_locator(
    warthog::domain::gridmap* map)
    : map_(map) //, jumplimit_(UINT32_MAX)
{ }

four_connected_jps_locator::~four_connected_jps_locator() { }

// Finds a jump point successor of node (x, y) in Direction d.
// Also given is the location of the goal node (goalx, goaly) for a particular
// search instance. If encountered, the goal node is always returned as a
// jump point successor.
//
// @return: the id of a jump point successor or INF32 if no jp exists.
void
four_connected_jps_locator::jump(
    jps::direction d, jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    double& jumpcost)
{
	switch(d)
	{
	case jps::NORTH:
		jump_north(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case jps::SOUTH:
		jump_south(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case jps::EAST:
		jump_east(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	case jps::WEST:
		jump_west(node_id, goal_id, jumpnode_id, jumpcost);
		break;
	default:
		break;
	}
}

void
four_connected_jps_locator::jump_north(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	uint32_t num_steps = 0;
	uint32_t mapw      = map_->width();

	jps_id jp_w_id;
	jps_id jp_e_id;
	double jp_w_cost;
	double jp_e_cost;

	jps_id next_id = node_id;
	while(true)
	{
		next_id.id -= mapw;
		num_steps++;

		// verify the next location is traversable
		if(!map_->get_label(next_id))
		{
			next_id = jps_id::none();
			break;
		}

		jump_east(next_id, goal_id, jp_e_id, jp_e_cost);
		if(!jp_e_id.is_none()) { break; }
		jump_west(next_id, goal_id, jp_w_id, jp_w_cost);
		if(!jp_w_id.is_none()) { break; }
	}

	jumpnode_id = next_id;
	jumpcost    = num_steps;

	// adjust num_steps if we stopped due to a deadend
	// (we return the distance to the last traversable tile)
	num_steps = !next_id.is_none() ? num_steps : num_steps - 1;
}

void
four_connected_jps_locator::jump_south(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	uint32_t num_steps = 0;
	uint32_t mapw      = map_->width();

	jps_id jp_w_id;
	jps_id jp_e_id;
	double jp_w_cost;
	double jp_e_cost;

	jps_id next_id = node_id;
	while(true)
	{
		next_id.id += mapw;
		num_steps++;

		// verify the next location is traversable
		if(!map_->get_label(next_id))
		{
			next_id = jps_id::none();
			break;
		}

		jump_east(next_id, goal_id, jp_e_id, jp_e_cost);
		if(!jp_e_id.is_none()) { break; }
		jump_west(next_id, goal_id, jp_w_id, jp_w_cost);
		if(!jp_w_id.is_none()) { break; }
	}
	jumpnode_id = next_id;
	jumpcost    = num_steps;

	// adjust num_steps if we stopped due to a deadend
	// (we return the distance to the last traversable tile)
	num_steps = !next_id.is_none() ? num_steps : num_steps - 1;
}

void
four_connected_jps_locator::jump_east(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	jumpnode_id = node_id;

	uint32_t neis[3] = {0, 0, 0};
	bool deadend     = false;

	jumpnode_id = node_id;
	while(true)
	{
		// read in tiles from 3 adjacent rows. the curent node
		// is in the low byte of the middle row
		map_->get_neighbours_32bit(jumpnode_id, neis);

		// identify forced neighbours and deadend tiles.
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
			// TODO: remove builtin
			uint32_t stop_pos = __builtin_ffs(stop_bits) - 1; // returns idx+1
			jumpnode_id.id   += stop_pos;
			deadend           = deadend_bits & (1 << stop_pos);
			break;
		}

		// jump to the last position in the cache. we do not jump past the end
		// in case the last tile from the row above or below is an obstacle.
		// Such a tile, followed by a non-obstacle tile, would yield a forced
		// neighbour that we don't want to miss.
		jumpnode_id.id += 31;
	}

	uint32_t num_steps = uint32_t{jumpnode_id} - uint32_t{node_id};
	uint32_t goal_dist = uint32_t{goal_id} - uint32_t{node_id};
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

void
four_connected_jps_locator::jump_west(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	bool deadend     = false;
	uint32_t neis[3] = {0, 0, 0};

	jumpnode_id = node_id;
	while(true)
	{
		// cache 32 tiles from three adjacent rows.
		// current tile is in the high byte of the middle row
		map_->get_neighbours_upper_32bit(jumpnode_id, neis);

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

	uint32_t num_steps = uint32_t{node_id} - uint32_t{jumpnode_id};
	uint32_t goal_dist = uint32_t{node_id} - uint32_t{goal_id};
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

} // namespace jps::jump
