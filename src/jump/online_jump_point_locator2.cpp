#include <jps/jump/online_jump_point_locator2.h>
#include <jps/search/jps.h>
#include <warthog/domain/gridmap.h>

#include <cassert>
#include <climits>

warthog::jps::online_jump_point_locator2::online_jump_point_locator2(
    warthog::domain::gridmap* map)
    : map_(map) //, jumplimit_(UINT32_MAX)
{
	rmap_ = create_rmap();
	current_node_id_ = current_rnode_id_ = warthog::INF32;
	current_goal_id_ = current_rgoal_id_ = warthog::INF32;
}

warthog::jps::online_jump_point_locator2::~online_jump_point_locator2()
{
	delete rmap_;
}

// create a copy of the grid map which is rotated by 90 degrees clockwise.
// this version will be used when jumping North or South.
warthog::domain::gridmap*
warthog::jps::online_jump_point_locator2::create_rmap()
{
	uint32_t maph = map_->header_height();
	uint32_t mapw = map_->header_width();
	uint32_t rmaph = mapw;
	uint32_t rmapw = maph;
	warthog::domain::gridmap* rmap
	    = new warthog::domain::gridmap(rmaph, rmapw);

	for(uint32_t x = 0; x < mapw; x++)
	{
		for(uint32_t y = 0; y < maph; y++)
		{
			uint32_t label = map_->get_label(map_->to_padded_id(x, y));
			uint32_t rx = ((rmapw - 1) - y);
			uint32_t ry = x;
			uint32_t rid = rmap->to_padded_id(rx, ry);
			rmap->set_label(rid, label);
		}
	}
	return rmap;
}

// Finds a jump point successor of node (x, y) in Direction d.
// Also given is the location of the goal node (goalx, goaly) for a particular
// search instance. If encountered, the goal node is always returned as a
// jump point successor.
//
// @return: the id of a jump point successor or warthog::INF if no jp exists.
void
warthog::jps::online_jump_point_locator2::jump(
    direction d, uint32_t node_id, uint32_t goal_id,
    std::vector<uint32_t>& jpoints, std::vector<warthog::cost_t>& costs)
{
	__jump_east_fp = &warthog::jps::online_jump_point_locator2::__jump_east;
	__jump_west_fp = &warthog::jps::online_jump_point_locator2::__jump_west;

	// cache node and goal ids so we don't need to convert all the time
	if(goal_id != current_goal_id_)
	{
		current_goal_id_ = goal_id;
		current_rgoal_id_ = map_id_to_rmap_id(goal_id);
	}

	if(node_id != current_node_id_)
	{
		current_node_id_ = node_id;
		current_rnode_id_ = map_id_to_rmap_id(node_id);
	}

	switch(d)
	{
	case NORTH:
		jump_north(jpoints, costs);
		break;
	case SOUTH:
		jump_south(jpoints, costs);
		break;
	case EAST:
		jump_east(jpoints, costs);
		break;
	case WEST:
		jump_west(jpoints, costs);
		break;
	case NORTHEAST:
		jump_northeast(jpoints, costs);
		break;
	case NORTHWEST:
		jump_northwest(jpoints, costs);
		break;
	case SOUTHEAST:
		jump_southeast(jpoints, costs);
		break;
	case SOUTHWEST:
		jump_southwest(jpoints, costs);
		break;
	default:
		break;
	}
}

void
warthog::jps::online_jump_point_locator2::jump_north(
    std::vector<uint32_t>& jpoints, std::vector<warthog::cost_t>& costs)
{
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;
	uint32_t jumpnode_id;
	warthog::cost_t jumpcost;

	__jump_north(rnode_id, rgoal_id, jumpnode_id, jumpcost, rmap_);

	if(jumpnode_id != warthog::INF32)
	{
		jumpnode_id = current_node_id_ - (uint32_t)(jumpcost)*map_->width();
		jpoints.push_back(jumpnode_id);
		costs.push_back(jumpcost);
	}
}

void
warthog::jps::online_jump_point_locator2::__jump_north(
    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// east when we use a version of the map rotated 90 degrees.
	(this->*(__jump_east_fp))(node_id, goal_id, jumpnode_id, jumpcost, mymap);
}

void
warthog::jps::online_jump_point_locator2::jump_south(
    std::vector<uint32_t>& jpoints, std::vector<warthog::cost_t>& costs)
{
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;
	uint32_t jumpnode_id;
	warthog::cost_t jumpcost;

	__jump_south(rnode_id, rgoal_id, jumpnode_id, jumpcost, rmap_);

	if(jumpnode_id != warthog::INF32)
	{
		jumpnode_id = current_node_id_ + (uint32_t)(jumpcost)*map_->width();
		jpoints.push_back(jumpnode_id);
		costs.push_back(jumpcost);
	}
}

void
warthog::jps::online_jump_point_locator2::__jump_south(
    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap)
{
	// jumping north in the original map is the same as jumping
	// west when we use a version of the map rotated 90 degrees.
	(this->*(__jump_west_fp))(node_id, goal_id, jumpnode_id, jumpcost, mymap);
}

void
warthog::jps::online_jump_point_locator2::jump_east(
    std::vector<uint32_t>& jpoints, std::vector<warthog::cost_t>& costs)
{
	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t jumpnode_id;
	warthog::cost_t jumpcost;

	(this->*(__jump_east_fp))(node_id, goal_id, jumpnode_id, jumpcost, map_);

	if(jumpnode_id != warthog::INF32)
	{
		jpoints.push_back(jumpnode_id);
		costs.push_back(jumpcost);
	}
}

void
warthog::jps::online_jump_point_locator2::__jump_east(
    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap)
{
	uint64_t neis[3] = {0, 0, 0};
	bool deadend = false;

	// read tiles from the grid:
	// - along the row of node_id
	// - from the row above node_id
	// - from the row below node_id
	// NB: the jump direction (here, EAST) corresponds to moving from the
	// low bit of the tileset and towards the high bit
	mymap->get_neighbours_64bit(node_id, neis);

	// extract the bit position of node_id in the tileset
	uint32_t bit_offset = node_id & 63;

	// look for tiles with forced neighbours in the rows above and below
	// A forced neighbour can be identified as a non-obstacle tile that
	// follows immediately  after an obstacle tile.
	// we ignore forced tiles which are at offsets >= bit_offset
	// (i.e., all tiles in {WEST of, above, below} the current location)
	uint64_t forced_bits = (~neis[0] << 1) & neis[0];
	forced_bits |= (~neis[2] << 1) & neis[2];
	forced_bits &= ~((1LL << bit_offset) | ((1LL << bit_offset) - 1));

	// look for obstacles tiles in the current row
	// we ignore obstacles at offsets > bit_offset
	uint64_t deadend_bits = ~neis[1];
	deadend_bits &= ~((1LL << bit_offset) - 1);

	// stop jumping if any forced or deadend locations are found
	uint64_t stop_bits = (forced_bits | deadend_bits);
	if(stop_bits)
	{
		// figure out how far we jumped (we count trailing zeroes)
		// we then subtract -1 because we want to know how many
		// steps from the bit offset to the stop bit
		int stop_pos = __builtin_ctzll(stop_bits);
		uint32_t num_steps = (stop_pos - bit_offset);

		// don't jump over the target
		uint32_t goal_dist = goal_id - node_id;
		if(num_steps > goal_dist)
		{
			jumpnode_id = goal_id;
			jumpcost = goal_dist;
			return;
		}

		// did we reach a jump point or a dead-end?
		deadend = (deadend_bits & (1LL << stop_pos));
		if(deadend)
		{
			jumpcost = num_steps - (1 && num_steps);
			jumpnode_id = warthog::INF32;
			return;
		}

		jumpnode_id = node_id + num_steps;
		jumpcost = num_steps;
		return;
	}

	// keep jumping. the procedure below is implemented
	// similarly to the above. but now the stride is a
	// fixed 64bit and the jumps are word-aligned.
	jumpnode_id = node_id + 64 - bit_offset;
	while(true)
	{
		// we need to forced neighbours might occur across
		// 64bit boundaries. to check for these we keep the
		// high-byte of the previous set of neighbours
		uint64_t high_ra = neis[0] >> 63;
		uint64_t high_rb = neis[2] >> 63;

		// read next 64bit set of tile data
		mymap->get_neighbours_64bit(jumpnode_id, neis);

		// identify forced neighbours and deadend tiles.
		uint64_t forced_bits = ~((neis[0] << 1) | high_ra) & neis[0];
		forced_bits |= ~((neis[2] << 1) | high_rb) & neis[2];
		uint64_t deadend_bits = ~neis[1];

		// stop if we found any forced or dead-end tiles
		uint64_t stop_bits = (forced_bits | deadend_bits);
		if(stop_bits)
		{
			int stop_pos = __builtin_ctzll(stop_bits);
			jumpnode_id += stop_pos;
			deadend = deadend_bits & (1LL << stop_pos);
			break;
		}
		jumpnode_id += 64;
	}

	// figure out how far we jumped
	uint32_t num_steps = jumpnode_id - node_id;

	// don't jump over the target
	uint32_t goal_dist = goal_id - node_id;
	if(num_steps > goal_dist)
	{
		jumpnode_id = goal_id;
		jumpcost = goal_dist;
		return;
	}

	// did we hit a dead-end?
	if(deadend)
	{
		// in this case we want to return the number of steps to
		// the last traversable tile (not to the obstacle)
		// need -1 to fix it.
		num_steps -= (1 && num_steps);
		jumpnode_id = warthog::INF32;
	}

	// return the number of steps to reach the jump point or deadend
	jumpcost = num_steps;
}

// analogous to ::jump_east
void
warthog::jps::online_jump_point_locator2::jump_west(
    std::vector<uint32_t>& jpoints, std::vector<warthog::cost_t>& costs)
{
	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t jumpnode_id;
	warthog::cost_t jumpcost;

	(this->*(__jump_west_fp))(node_id, goal_id, jumpnode_id, jumpcost, map_);

	if(jumpnode_id != warthog::INF32)
	{
		jpoints.push_back(jumpnode_id);
		costs.push_back(jumpcost);
	}
}

void
warthog::jps::online_jump_point_locator2::__jump_west(
    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap)
{
	bool deadend = false;
	uint64_t neis[3] = {0, 0, 0};

	// read sets of 64 tiles from the grid:
	// - along the row of node_id
	// - from the row above node_id
	// - from the row below node_id
	// NB: the jump direction (here, WEST) corresponds to moving from the
	// high bit of the tileset and towards the low bit
	mymap->get_neighbours_64bit(node_id, neis);

	// extract the bit position of node_id in the tileset
	uint32_t bit_offset = node_id & 63;

	// look for tiles with forced neighbours in the rows above and below
	// A forced neighbour can be identified as a non-obstacle tile that
	// follows immediately  after an obstacle tile.
	// NB: we ignore forced tiles which are at offsets <= bit_offset
	// (i.e., all tiles in {EAST of, above, below} the current location)
	uint64_t forced_bits = (~neis[0] >> 1) & neis[0];
	forced_bits |= (~neis[2] >> 1) & neis[2];
	forced_bits &= ~(UINT64_MAX << bit_offset);

	// look for obstacles tiles in the current row
	// NB: we ignore obstacles at offsets < bit_offset
	uint64_t deadend_bits = ~neis[1];
	deadend_bits &= (1LL << bit_offset) | ((1LL << bit_offset) - 1);

	// stop jumping if any forced or deadend locations are found
	uint64_t stop_bits = (forced_bits | deadend_bits);
	if(stop_bits)
	{
		// figure out how far we jumped (we count leading zeroes)
		// we then subtract -1 because we want to know how many
		// steps from the bit offset to the stop bit
		uint64_t stop_pos = __builtin_clzll(stop_bits);
		uint32_t num_steps = (stop_pos - (63 - bit_offset));

		// don't jump over the target
		uint32_t goal_dist = node_id - goal_id;
		if(num_steps > goal_dist)
		{
			jumpnode_id = goal_id;
			jumpcost = goal_dist;
			return;
		}

		// did we reach a jump point or a dead-end?
		deadend = deadend_bits & (0x8000000000000000 >> stop_pos);
		if(deadend)
		{
			// number of steps to reach the deadend tile is not
			// correct here since we just inverted neis[1] and then
			// counted leading zeroes. need -1 to fix it.
			jumpcost = num_steps - (1 && num_steps);
			jumpnode_id = warthog::INF32;
			return;
		}

		jumpnode_id = node_id - num_steps;
		jumpcost = num_steps;
		return;
	}

	// keep jumping. the procedure below is implemented
	// similarly to the above. but now the stride is a
	// fixed 64bit and the jumps are word-aligned.
	jumpnode_id = node_id - (bit_offset + 1);
	while(true)
	{
		// we need to forced neighbours might occur across
		// 64bit boundaries. to check for these we keep the
		// low-byte of the previous set of neighbours
		uint64_t low_ra = neis[0] << 63;
		uint64_t low_rb = neis[2] << 63;

		// read next 64 bit set of tile data
		mymap->get_neighbours_64bit(jumpnode_id, neis);

		// identify forced and dead-end nodes
		uint64_t forced_bits = ~((neis[0] >> 1) | low_ra) & neis[0];
		forced_bits |= ~((neis[2] >> 1) | low_rb) & neis[2];
		uint64_t deadend_bits = ~neis[1];

		// stop if we encounter any forced or deadend nodes
		uint64_t stop_bits = (forced_bits | deadend_bits);
		if(stop_bits)
		{
			uint64_t stop_pos = (uint64_t)__builtin_clzll(stop_bits);
			jumpnode_id -= stop_pos;
			deadend = deadend_bits & (0x8000000000000000 >> stop_pos);
			break;
		}
		jumpnode_id -= 64;
	}

	// figure out how far we jumped
	uint32_t num_steps = node_id - jumpnode_id;

	// don't jump over the target
	uint32_t goal_dist = node_id - goal_id;
	if(num_steps > goal_dist)
	{
		jumpnode_id = goal_id;
		jumpcost = goal_dist;
		return;
	}

	// did we hit a dead-end?
	if(deadend)
	{
		// in this case we want to return the number of steps to
		// the last traversable tile (not to the obstacle)
		// need -1 to fix it.
		num_steps -= (1 && num_steps);
		jumpnode_id = warthog::INF32;
	}

	// return the number of steps to reach the jump point or deadend
	jumpcost = num_steps;
}

void
warthog::jps::online_jump_point_locator2::jump_northeast(
    std::vector<uint32_t>& jpoints, std::vector<warthog::cost_t>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	warthog::cost_t jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. node_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	if((neis & 1542) != 1542) { return; }

	while(node_id != warthog::INF32)
	{
		__jump_northeast(
		    node_id, rnode_id, goal_id, rgoal_id, jumpnode_id, jumpcost,
		    jp1_id, jp1_cost, jp2_id, jp2_cost);

		if(jp1_id != warthog::INF32)
		{
			jp1_id = node_id - (uint32_t)(jp1_cost)*map_->width();
			jpoints.push_back(jp1_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
			if(jp2_cost == 0) { break; } // no corner cutting
		}

		if(jp2_id != warthog::INF32)
		{
			jpoints.push_back(jp2_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
			if(jp1_cost == 0) { break; } // no corner cutting
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
warthog::jps::online_jump_point_locator2::__jump_northeast(
    uint32_t& node_id, uint32_t& rnode_id, uint32_t goal_id, uint32_t rgoal_id,
    uint32_t& jumpnode_id, warthog::cost_t& jumpcost, uint32_t& jp_id1,
    warthog::cost_t& cost1, uint32_t& jp_id2, warthog::cost_t& cost2)
{
	uint32_t num_steps = 0;

	// jump a single step at a time (no corner cutting)
	uint32_t rmapw = rmap_->width();
	uint32_t mapw = map_->width();
	while(true)
	{
		num_steps++;
		node_id = node_id - mapw + 1;
		rnode_id = rnode_id + rmapw + 1;

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		__jump_north(rnode_id, rgoal_id, jp_id1, cost1, rmap_);
		(this->*(__jump_east_fp))(node_id, goal_id, jp_id2, cost2, map_);
		if((jp_id1 & jp_id2) != warthog::INF32) { break; }

		// couldn't move in a straight dir; next step is an obstacle
		if(!((uint64_t)cost1 && (uint64_t)cost2))
		{
			node_id = jp_id1 = jp_id2 = warthog::INF32;
			break;
		}
	}
	jumpnode_id = node_id;
	jumpcost = num_steps * warthog::DBL_ROOT_TWO;
}

void
warthog::jps::online_jump_point_locator2::jump_northwest(
    std::vector<uint32_t>& jpoints, std::vector<warthog::cost_t>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	warthog::cost_t jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. node_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	if((neis & 771) != 771) { return; }

	while(node_id != warthog::INF32)
	{
		__jump_northwest(
		    node_id, rnode_id, goal_id, rgoal_id, jumpnode_id, jumpcost,
		    jp1_id, jp1_cost, jp2_id, jp2_cost);

		if(jp1_id != warthog::INF32)
		{
			jp1_id = node_id - (uint32_t)(jp1_cost)*map_->width();
			jpoints.push_back(jp1_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
			if(jp2_cost == 0) { break; } // no corner cutting
		}

		if(jp2_id != warthog::INF32)
		{
			jpoints.push_back(jp2_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
			if(jp1_cost == 0) { break; } // no corner cutting
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
warthog::jps::online_jump_point_locator2::__jump_northwest(
    uint32_t& node_id, uint32_t& rnode_id, uint32_t goal_id, uint32_t rgoal_id,
    uint32_t& jumpnode_id, warthog::cost_t& jumpcost, uint32_t& jp_id1,
    warthog::cost_t& cost1, uint32_t& jp_id2, warthog::cost_t& cost2)

{
	uint32_t num_steps = 0;

	// jump a single step at a time (no corner cutting)
	uint32_t rmapw = rmap_->width();
	uint32_t mapw = map_->width();
	while(true)
	{
		num_steps++;
		node_id = node_id - mapw - 1;
		rnode_id = rnode_id - (rmapw - 1);

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		__jump_north(rnode_id, rgoal_id, jp_id1, cost1, rmap_);
		(this->*(__jump_west_fp))(node_id, goal_id, jp_id2, cost2, map_);
		if((jp_id1 & jp_id2) != warthog::INF32) { break; }

		// couldn't move in a straight dir; next step is an obstacle
		if(!((uint64_t)cost1 && (uint64_t)cost2))
		{
			node_id = jp_id1 = jp_id2 = warthog::INF32;
			break;
		}
	}
	jumpnode_id = node_id;
	jumpcost = num_steps * warthog::DBL_ROOT_TWO;
}

void
warthog::jps::online_jump_point_locator2::jump_southeast(
    std::vector<uint32_t>& jpoints, std::vector<warthog::cost_t>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	warthog::cost_t jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	if((neis & 394752) != 394752) { return; }

	while(node_id != warthog::INF32)
	{
		__jump_southeast(
		    node_id, rnode_id, goal_id, rgoal_id, jumpnode_id, jumpcost,
		    jp1_id, jp1_cost, jp2_id, jp2_cost);

		if(jp1_id != warthog::INF32)
		{
			jp1_id = node_id + (uint32_t)(jp1_cost)*map_->width();
			jpoints.push_back(jp1_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
			if(jp2_cost == 0) { break; } // no corner cutting
		}

		if(jp2_id != warthog::INF32)
		{
			jpoints.push_back(jp2_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
			if(jp1_cost == 0) { break; } // no corner cutting
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
warthog::jps::online_jump_point_locator2::__jump_southeast(
    uint32_t& node_id, uint32_t& rnode_id, uint32_t goal_id, uint32_t rgoal_id,
    uint32_t& jumpnode_id, warthog::cost_t& jumpcost, uint32_t& jp_id1,
    warthog::cost_t& cost1, uint32_t& jp_id2, warthog::cost_t& cost2)

{
	uint32_t num_steps = 0;

	// jump a single step at a time (no corner cutting)
	uint32_t rmapw = rmap_->width();
	uint32_t mapw = map_->width();
	while(true)
	{
		num_steps++;
		node_id = node_id + mapw + 1;
		rnode_id = rnode_id + rmapw - 1;

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		__jump_south(rnode_id, rgoal_id, jp_id1, cost1, rmap_);
		(this->*(__jump_east_fp))(node_id, goal_id, jp_id2, cost2, map_);
		if((jp_id1 & jp_id2) != warthog::INF32) { break; }

		// couldn't move in a straight dir; next step is an obstacle
		if(!((uint64_t)cost1 && (uint64_t)cost2))
		{
			node_id = jp_id1 = jp_id2 = warthog::INF32;
			break;
		}
	}
	jumpnode_id = node_id;
	jumpcost = num_steps * warthog::DBL_ROOT_TWO;
}

void
warthog::jps::online_jump_point_locator2::jump_southwest(
    std::vector<uint32_t>& jpoints, std::vector<warthog::cost_t>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	warthog::cost_t jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early termination (first step is invalid)
	if((neis & 197376) != 197376) { return; }

	while(node_id != warthog::INF32)
	{
		__jump_southwest(
		    node_id, rnode_id, goal_id, rgoal_id, jumpnode_id, jumpcost,
		    jp1_id, jp1_cost, jp2_id, jp2_cost);

		if(jp1_id != warthog::INF32)
		{
			jp1_id = node_id + (uint32_t)(jp1_cost)*map_->width();
			jpoints.push_back(jp1_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
			if(jp2_cost == 0) { break; }
		}

		if(jp2_id != warthog::INF32)
		{
			jpoints.push_back(jp2_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
			if(jp1_cost == 0) { break; }
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
warthog::jps::online_jump_point_locator2::__jump_southwest(
    uint32_t& node_id, uint32_t& rnode_id, uint32_t goal_id, uint32_t rgoal_id,
    uint32_t& jumpnode_id, warthog::cost_t& jumpcost, uint32_t& jp_id1,
    warthog::cost_t& cost1, uint32_t& jp_id2, warthog::cost_t& cost2)
{
	// jump a single step (no corner cutting)
	uint32_t num_steps = 0;
	uint32_t mapw = map_->width();
	uint32_t rmapw = rmap_->width();
	while(true)
	{
		num_steps++;
		node_id = node_id + mapw - 1;
		rnode_id = rnode_id - (rmapw + 1);

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		__jump_south(rnode_id, rgoal_id, jp_id1, cost1, rmap_);
		(this->*(__jump_west_fp))(node_id, goal_id, jp_id2, cost2, map_);
		if((jp_id1 & jp_id2) != warthog::INF32) { break; }

		// couldn't move in a straight dir; next step is an obstacle
		if(!((uint64_t)cost1 && (uint64_t)cost2))
		{
			node_id = jp_id1 = jp_id2 = warthog::INF32;
			break;
		}
	}
	jumpnode_id = node_id;
	jumpcost = num_steps * warthog::DBL_ROOT_TWO;
}
