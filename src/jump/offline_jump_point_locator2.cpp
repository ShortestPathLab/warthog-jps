#include <jps/jump/offline_jump_point_locator2.h>
#include <jps/jump/online_jump_point_locator.h>
#include <warthog/domain/gridmap.h>

#include <assert.h>
#include <cstring>
#include <inttypes.h>
#include <stdio.h>

namespace jps::jump
{

offline_jump_point_locator2::offline_jump_point_locator2(
    warthog::domain::gridmap* map)
    : map_(map)
{
	if(map_->padded_mapsize() > ((1 << 23) - 1))
	{
		// search nodes are stored as 32bit quantities.
		// bit 0 stores the expansion status
		// bits 1-23 store the unique node id
		// bits 24-31 store the direction from the parent
		std::cerr << "map size too big for this implementation of JPS+."
		          << " aborting." << std::endl;
		exit(1);
	}
	preproc();
}

offline_jump_point_locator2::~offline_jump_point_locator2()
{
	delete[] db_;
}

void
offline_jump_point_locator2::preproc()
{
	if(load(map_->filename())) { return; }

	dbsize_ = 8 * map_->padded_mapsize();
	db_     = new uint16_t[dbsize_];
	for(uint32_t i = 0; i < dbsize_; i++)
		db_[i] = 0;

	online_jump_point_locator jpl(map_);
	for(uint32_t y = 0; y < map_->header_height(); y++)
	{
		for(uint32_t x = 0; x < map_->header_width(); x++)
		{
			jps_id mapid = jps_id{map_->to_padded_id_from_unpadded(x, y)};
			//			std::cout << mapid << " ";
			for(uint32_t i = 0; i < 8; i++)
			{
				direction dir = (direction)(1 << i);
				//				std::cout << dir << ":
				//";
				jps_id jumpnode_id;
				double jumpcost;
				jpl.jump(dir, mapid, jps_id::none(), jumpnode_id, jumpcost);

				// convert from cost to number of steps
				if(dir > 8)
				{
					jumpcost = jumpcost * warthog::DBL_ONE_OVER_ROOT_TWO;
				}
				uint32_t num_steps = (uint16_t)floor((jumpcost + 0.5));
				//				std::cout << (jumpnode_id == INF ? 0 :
				// num_steps) << " ";

				// set the leading bit if the jump leads to a dead-end
				if(jumpnode_id.is_none()) { db_[mapid.id * 8 + i] |= 32768; }

				// truncate jump cost so we can fit the label into a single
				// byte
				// if(num_steps > 32767)
				//{
				//	num_steps = 32767;
				//	jumpnode_id = 0;
				//}

				db_[mapid.id * 8 + i] |= num_steps;

				if(num_steps > 32768)
				{
					std::cerr << "label overflow; maximum jump distance "
					             "exceeded. aborting\n";
					exit(1);
				}
			}
			//			std::cout << std::endl;
		}
	}

	save(map_->filename());
}

bool
offline_jump_point_locator2::load(const char* filename)
{
	char fname[256];
	strcpy(fname, filename);
	strcat(fname, ".jps+");
	FILE* f = fopen(fname, "rb");
	std::cerr << "loading " << fname << "... ";
	if(f == NULL)
	{
		std::cerr << "no dice. oh well. keep going.\n" << std::endl;
		return false;
	}

	fread(&dbsize_, sizeof(dbsize_), 1, f);
	std::cerr << "#labels=" << dbsize_ << std::endl;

	db_ = new uint16_t[dbsize_];
	fread(db_, sizeof(uint16_t), dbsize_, f);
	fclose(f);
	return true;
}

void
offline_jump_point_locator2::save(const char* filename)
{
	char fname[256];
	strcpy(fname, filename);
	strcat(fname, ".jps+");
	std::cerr << "saving to file " << fname << "; nodes=" << dbsize_
	          << " size: " << sizeof(db_[0]) << std::endl;

	FILE* f = fopen(fname, "wb");
	if(f == NULL)
	{
		std::cerr << "err; cannot write jump-point graph to file " << fname
		          << ". oh well. try to keep going.\n"
		          << std::endl;
		return;
	}

	fwrite(&dbsize_, sizeof(dbsize_), 1, f);
	fwrite(db_, sizeof(*db_), dbsize_, f);
	fclose(f);
	std::cerr << "jump table saved to disk. file=" << fname << std::endl;
}

void
offline_jump_point_locator2::jump(
    direction d, jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
    vec_jps_cost& costs)
{
	switch(d)
	{
	case NORTH:
		jump_north(node_id, goal_id, 0, neighbours, costs);
		break;
	case SOUTH:
		jump_south(node_id, goal_id, 0, neighbours, costs);
		break;
	case EAST:
		jump_east(node_id, goal_id, 0, neighbours, costs);
		break;
	case WEST:
		jump_west(node_id, goal_id, 0, neighbours, costs);
		break;
	case NORTHEAST:
		jump_northeast(node_id, goal_id, neighbours, costs);
		break;
	case NORTHWEST:
		jump_northwest(node_id, goal_id, neighbours, costs);
		break;
	case SOUTHEAST:
		jump_southeast(node_id, goal_id, neighbours, costs);
		break;
	case SOUTHWEST:
		jump_southwest(node_id, goal_id, neighbours, costs);
		break;
	default:
		break;
	}
}

void
offline_jump_point_locator2::jump_northwest(
    jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
    vec_jps_cost& costs)

{
	uint16_t label           = 0;
	uint16_t num_steps       = 0;
	uint32_t mapw            = map_->width();
	uint32_t diag_step_delta = (mapw + 1);

	// keep jumping until we hit a dead-end. take note of the
	// points reachable by a vertical or horizontal jump from
	// each intermediate location that we reach diagonally.
	jps_id jump_from = node_id;

	// step diagonally to an intermediate location jump_from
	label      = db_[8 * jump_from.id + 5];
	num_steps += label & 32767;
	jump_from  = jps_id(node_id.id - num_steps * diag_step_delta);
	while(!(label & 32768))
	{
		// north of jump_from
		uint16_t label_straight1 = db_[8 * jump_from.id];
		if(!(label_straight1 & 32768))
		{
			uint32_t jp_cost = (label_straight1 & 32767);
			jps_id jp_id     = jps_id(jump_from.id - mapw * jp_cost);
			neighbours.push_back(jp_id);
			costs.push_back(jp_cost + num_steps * warthog::DBL_ROOT_TWO);
		}
		// west of jump_from
		uint16_t label_straight2
		    = db_[8 * jump_from.id + 3]; // west of next jp
		if(!(label_straight2 & 32768))
		{
			uint32_t jp_cost = (label_straight2 & 32767);
			jps_id jp_id     = jps_id(jump_from.id - jp_cost);
			neighbours.push_back(jp_id);
			costs.push_back(jp_cost + num_steps * warthog::DBL_ROOT_TWO);
		}
		label      = db_[8 * jump_from.id + 5];
		num_steps += label & 32767;
		jump_from  = jps_id(node_id.id - num_steps * diag_step_delta);
	}

	// goal test (so many div ops! and branches! how ugly!)
	if((node_id.id - goal_id.id)
	   < map_->padded_mapsize()) // heading toward the goal?
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (ny - gy);
		uint32_t xdelta = (nx - gx);
		if(xdelta < mapw && ydelta < map_->height())
		{
			if(ydelta < xdelta && ydelta <= num_steps)
			{
				jps_id jp_id   = jps_id(node_id.id - diag_step_delta * ydelta);
				double jp_cost = warthog::DBL_ROOT_TWO * ydelta;
				jump_west(jp_id, goal_id, jp_cost, neighbours, costs);
			}
			else if(xdelta <= num_steps)
			{
				jps_id jp_id   = jps_id(node_id.id - diag_step_delta * xdelta);
				double jp_cost = warthog::DBL_ROOT_TWO * xdelta;
				jump_north(jp_id, goal_id, jp_cost, neighbours, costs);
			}
		}
	}
}

void
offline_jump_point_locator2::jump_northeast(
    jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
    vec_jps_cost& costs)
{
	uint16_t label           = 0;
	uint16_t num_steps       = 0;
	uint32_t mapw            = map_->width();
	uint32_t diag_step_delta = (mapw - 1);

	jps_id jump_from = node_id;
	// step diagonally to an intermediate location jump_from
	label      = db_[8 * jump_from.id + 4];
	num_steps += label & 32767;
	jump_from  = jps_id(node_id.id - num_steps * diag_step_delta);
	while(!(label & 32768))
	{

		// north of jump_from
		uint16_t label_straight1 = db_[8 * jump_from.id];
		if(!(label_straight1 & 32768))
		{
			uint32_t jp_cost = (label_straight1 & 32767);
			jps_id jp_id     = jps_id(jump_from.id - mapw * jp_cost);
			neighbours.push_back(jp_id);
			costs.push_back(jp_cost + num_steps * warthog::DBL_ROOT_TWO);
		}
		// east of jump_from
		uint16_t label_straight2 = db_[8 * jump_from.id + 2];
		if(!(label_straight2 & 32768))
		{
			uint32_t jp_cost = (label_straight2 & 32767);
			jps_id jp_id     = jps_id(jump_from.id + jp_cost);
			neighbours.push_back(jp_id);
			costs.push_back(jp_cost + num_steps * warthog::DBL_ROOT_TWO);
		}
		label      = db_[8 * jump_from.id + 4];
		num_steps += label & 32767;
		jump_from  = jps_id(node_id.id - num_steps * diag_step_delta);
	}

	// goal test (so many div ops! and branches! how ugly!)
	if((node_id.id - goal_id.id)
	   < map_->padded_mapsize()) // heading toward the goal?
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (ny - gy);
		uint32_t xdelta = (gx - nx);
		if(xdelta < mapw && ydelta < map_->height())
		{
			if(ydelta < xdelta && ydelta <= num_steps)
			{
				jps_id jp_id   = jps_id(node_id.id - diag_step_delta * ydelta);
				double jp_cost = warthog::DBL_ROOT_TWO * ydelta;
				jump_east(jp_id, goal_id, jp_cost, neighbours, costs);
			}
			else if(xdelta <= num_steps)
			{
				jps_id jp_id   = jps_id(node_id.id - diag_step_delta * xdelta);
				double jp_cost = warthog::DBL_ROOT_TWO * xdelta;
				jump_north(jp_id, goal_id, jp_cost, neighbours, costs);
			}
		}
	}
}

void
offline_jump_point_locator2::jump_southwest(
    jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
    vec_jps_cost& costs)
{
	uint32_t mapw            = map_->width();
	uint32_t diag_step_delta = (mapw - 1);
	uint16_t label, num_steps;
	label = num_steps = 0;

	jps_id jump_from = node_id;
	// step diagonally to an intermediate location jump_from
	label      = db_[8 * jump_from.id + 7];
	num_steps += label & 32767;
	jump_from  = jps_id(node_id.id + num_steps * diag_step_delta);
	while(!(label & 32768))
	{
		// south of jump_from
		uint16_t label_straight1 = db_[8 * jump_from.id + 1];
		if(!(label_straight1 & 32768))
		{
			uint32_t jp_cost = (label_straight1 & 32767);
			jps_id jp_id     = jps_id(jump_from.id + mapw * jp_cost);
			neighbours.push_back(jp_id);
			costs.push_back(jp_cost + num_steps * warthog::DBL_ROOT_TWO);
		}
		// west of jump_from
		uint16_t label_straight2 = db_[8 * jump_from.id + 3];
		if(!(label_straight2 & 32768))
		{
			uint32_t jp_cost = (label_straight2 & 32767);
			jps_id jp_id     = jps_id(jump_from.id - jp_cost);
			neighbours.push_back(jp_id);
			costs.push_back(jp_cost + num_steps * warthog::DBL_ROOT_TWO);
		}
		label      = db_[8 * jump_from.id + 7];
		num_steps += label & 32767;
		jump_from  = jps_id(node_id.id + num_steps * diag_step_delta);
	}

	// goal test (so many div ops! and branches! how ugly!)
	if((goal_id.id - node_id.id)
	   < map_->padded_mapsize()) // heading toward the goal?
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (gy - ny);
		uint32_t xdelta = (nx - gx);
		if(xdelta < mapw && ydelta < map_->height())
		{
			if(ydelta < xdelta && ydelta <= num_steps)
			{
				jps_id jp_id   = jps_id(node_id.id + diag_step_delta * ydelta);
				double jp_cost = warthog::DBL_ROOT_TWO * ydelta;
				jump_west(jp_id, goal_id, jp_cost, neighbours, costs);
			}
			else if(xdelta <= num_steps)
			{
				jps_id jp_id   = jps_id(node_id.id + diag_step_delta * xdelta);
				double jp_cost = warthog::DBL_ROOT_TWO * xdelta;
				jump_south(jp_id, goal_id, jp_cost, neighbours, costs);
			}
		}
	}
}

void
offline_jump_point_locator2::jump_southeast(
    jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
    vec_jps_cost& costs)

{
	uint16_t label           = 0;
	uint16_t num_steps       = 0;
	uint32_t mapw            = map_->width();
	uint32_t diag_step_delta = (mapw + 1);

	jps_id jump_from = node_id;

	// step diagonally to an intermediate location jump_from
	label      = db_[8 * jump_from.id + 6];
	num_steps += label & 32767;
	jump_from  = jps_id(node_id.id + num_steps * diag_step_delta);
	while(!(label & 32768))
	{
		// south of jump_from
		uint16_t label_straight1 = db_[8 * jump_from.id + 1];
		if(!(label_straight1 & 32768))
		{
			uint32_t jp_cost = (label_straight1 & 32767);
			jps_id jp_id     = jps_id(jump_from.id + mapw * jp_cost);
			neighbours.push_back(jp_id);
			costs.push_back(jp_cost + num_steps * warthog::DBL_ROOT_TWO);
		}
		// east of jump_from
		uint16_t label_straight2 = db_[8 * jump_from.id + 2];
		if(!(label_straight2 & 32768))
		{
			uint32_t jp_cost = (label_straight2 & 32767);
			jps_id jp_id     = jps_id(jump_from.id + jp_cost);
			neighbours.push_back(jp_id);
			costs.push_back(jp_cost + num_steps * warthog::DBL_ROOT_TWO);
		}
		// step diagonally to an intermediate location jump_from
		label      = db_[8 * jump_from.id + 6];
		num_steps += label & 32767;
		jump_from  = jps_id(node_id.id + num_steps * diag_step_delta);
	}

	// goal test (so many div ops! and branches! how ugly!)
	if((goal_id.id - node_id.id)
	   < map_->padded_mapsize()) // heading toward the goal?
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (gy - ny);
		uint32_t xdelta = (gx - nx);
		if(xdelta < mapw && ydelta < map_->height())
		{
			if(ydelta < xdelta && ydelta <= num_steps)
			{
				jps_id jp_id   = jps_id(node_id.id + diag_step_delta * ydelta);
				double jp_cost = warthog::DBL_ROOT_TWO * ydelta;
				jump_east(jp_id, goal_id, jp_cost, neighbours, costs);
			}
			else if(xdelta <= num_steps)
			{
				jps_id jp_id   = jps_id(node_id.id + diag_step_delta * xdelta);
				double jp_cost = warthog::DBL_ROOT_TWO * xdelta;
				jump_south(jp_id, goal_id, jp_cost, neighbours, costs);
			}
		}
	}
}

void
offline_jump_point_locator2::jump_north(
    jps_id node_id, jps_id goal_id, double cost_to_node_id,
    vec_jps_id& neighbours, vec_jps_cost& costs)
{
	uint16_t label     = db_[8 * node_id.id];
	uint16_t num_steps = label & 32767;

	// do not jump over the goal
	uint32_t id_delta   = num_steps * map_->width();
	uint32_t goal_delta = node_id.id - goal_id.id;
	if(id_delta >= goal_delta)
	{
		uint32_t gx = goal_id.id % map_->width();
		uint32_t nx = node_id.id % map_->width();
		if(nx == gx)
		{
			neighbours.push_back(goal_id);
			costs.push_back((goal_delta / map_->width()) + cost_to_node_id);
			return;
		}
	}

	// return the jump point at hand (if it isn't sterile)
	if(!(label & 32768))
	{
		jps_id jp_id = jps_id(node_id.id - id_delta);
		neighbours.push_back(jp_id);
		costs.push_back(num_steps + cost_to_node_id);
	}
}

void
offline_jump_point_locator2::jump_south(
    jps_id node_id, jps_id goal_id, double cost_to_node_id,
    vec_jps_id& neighbours, vec_jps_cost& costs)
{
	uint16_t label     = db_[8 * node_id.id + 1];
	uint16_t num_steps = label & 32767;

	// do not jump over the goal
	uint32_t id_delta   = num_steps * map_->width();
	uint32_t goal_delta = goal_id.id - node_id.id;
	if(id_delta >= goal_delta)
	{
		uint32_t gx = goal_id.id % map_->width();
		uint32_t nx = node_id.id % map_->width();
		if(nx == gx)
		{
			neighbours.push_back(goal_id);
			costs.push_back((goal_delta / map_->width()) + cost_to_node_id);
			return;
		}
	}

	// return the jump point at hand (if it isn't sterile)
	if(!(label & 32768))
	{
		jps_id jp_id = jps_id(node_id.id + id_delta);
		neighbours.push_back(jp_id);
		costs.push_back(num_steps + cost_to_node_id);
	}
}

void
offline_jump_point_locator2::jump_east(
    jps_id node_id, jps_id goal_id, double cost_to_node_id,
    vec_jps_id& neighbours, vec_jps_cost& costs)
{
	uint16_t label     = db_[8 * node_id.id + 2];
	uint32_t num_steps = label & 32767;

	// do not jump over the goal
	uint32_t goal_delta = goal_id.id - node_id.id;
	if(num_steps >= goal_delta)
	{
		neighbours.push_back(goal_id);
		costs.push_back(goal_delta + cost_to_node_id);
		return;
	}

	// return the jump point at hand
	if(!(label & 32768))
	{
		jps_id jp_id = jps_id(node_id.id + num_steps);
		neighbours.push_back(jp_id);
		costs.push_back(num_steps + cost_to_node_id);
	}
}

void
offline_jump_point_locator2::jump_west(
    jps_id node_id, jps_id goal_id, double cost_to_node_id,
    vec_jps_id& neighbours, vec_jps_cost& costs)
{
	uint16_t label     = db_[8 * node_id.id + 3];
	uint32_t num_steps = label & 32767;

	// do not jump over the goal
	uint32_t goal_delta = node_id.id - goal_id.id;
	if(num_steps >= goal_delta)
	{
		neighbours.push_back(goal_id);
		costs.push_back(goal_delta + cost_to_node_id);
		return;
	}

	// return the jump point at hand
	if(!(label & 32768))
	{
		jps_id jp_id = jps_id(node_id.id - num_steps);
		neighbours.push_back(jp_id);
		costs.push_back(num_steps + cost_to_node_id);
	}
}

} // namespace jps::jump
