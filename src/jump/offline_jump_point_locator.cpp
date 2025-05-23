#include <jps/jump/offline_jump_point_locator.h>
#include <jps/jump/online_jump_point_locator.h>
#include <warthog/domain/gridmap.h>

#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace jps::jump
{

offline_jump_point_locator::offline_jump_point_locator(
    warthog::domain::gridmap* map)
    : map_(map)
{
	preproc();
}

offline_jump_point_locator::~offline_jump_point_locator()
{
	delete[] db_;
}

void
offline_jump_point_locator::preproc()
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
offline_jump_point_locator::load(const char* filename)
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
offline_jump_point_locator::save(const char* filename)
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
	std::cerr << "jump-point graph saved to disk. file=" << fname << std::endl;
}

void
offline_jump_point_locator::jump(
    direction d, jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
    double& jumpcost)
{
	current_ = max_ = 0;
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
offline_jump_point_locator::jump_northwest(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	uint32_t mapw      = map_->width();
	uint16_t label     = db_[8 * node_id.id + 5];
	uint16_t num_steps = label & 32767;

	// goal test (so many div ops! and branches! how ugly!)
	uint32_t id_delta = (mapw + 1) * num_steps;
	if(node_id.id - goal_id.id
	   < map_->padded_mapsize()) // heading toward the goal?
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (ny - gy);
		uint32_t xdelta = (nx - gx);
		if(xdelta < mapw && ydelta < map_->height())
		{
			jumpnode_id = jps_id::none();
			uint32_t nid, steps_to_nid;
			if(ydelta < xdelta && ydelta <= num_steps)
			{
				steps_to_nid = ydelta;
				nid          = node_id.id - (mapw + 1) * steps_to_nid;
				jump_west(jps_id{nid}, goal_id, jumpnode_id, jumpcost);
				if(jumpnode_id == goal_id)
				{
					jumpnode_id = goal_id;
					jumpcost = steps_to_nid * warthog::DBL_ROOT_TWO + jumpcost;
					return;
				}
			}
			else if(xdelta <= num_steps)
			{
				steps_to_nid = xdelta;
				nid          = node_id.id - (mapw + 1) * steps_to_nid;
				jump_north(jps_id{nid}, goal_id, jumpnode_id, jumpcost);
				if(jumpnode_id == goal_id)
				{
					jumpnode_id = goal_id;
					jumpcost = steps_to_nid * warthog::DBL_ROOT_TWO + jumpcost;
					return;
				}
			}
		}
	}

	// return the jump point; but only if it isn't sterile
	jumpnode_id = jps_id(node_id.id - id_delta);
	jumpcost    = num_steps * warthog::DBL_ROOT_TWO;
	if(label & 32768) { jumpnode_id = jps_id::none(); }
}

void
offline_jump_point_locator::jump_northeast(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	uint16_t label     = db_[8 * node_id.id + 4];
	uint16_t num_steps = label & 32767;
	uint32_t mapw      = map_->width();

	// goal test (so many div ops! and branches! how ugly!)
	uint32_t id_delta = (mapw - 1) * num_steps;
	if((node_id.id - goal_id.id) < map_->padded_mapsize())
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (ny - gy);
		uint32_t xdelta = (gx - nx);
		if(xdelta < mapw && ydelta < map_->height())
		{
			jumpnode_id = jps_id::none();
			uint32_t nid, steps_to_nid;
			if(ydelta < xdelta && ydelta <= num_steps)
			{
				steps_to_nid = ydelta;
				nid          = node_id.id - (mapw - 1) * steps_to_nid;
				jump_east(jps_id(nid), goal_id, jumpnode_id, jumpcost);
				if(jumpnode_id == goal_id)
				{
					jumpnode_id = goal_id;
					jumpcost = steps_to_nid * warthog::DBL_ROOT_TWO + jumpcost;
					return;
				}
			}
			else if(xdelta <= num_steps)
			{
				steps_to_nid = xdelta;
				nid          = node_id.id - (mapw - 1) * steps_to_nid;
				jump_north(jps_id(nid), goal_id, jumpnode_id, jumpcost);
				if(jumpnode_id == goal_id)
				{
					jumpnode_id = goal_id;
					jumpcost = steps_to_nid * warthog::DBL_ROOT_TWO + jumpcost;
					return;
				}
			}
		}
	}

	// return the jump point; but only if it isn't sterile
	jumpnode_id = jps_id(node_id.id - id_delta);
	jumpcost    = num_steps * warthog::DBL_ROOT_TWO;
	if(label & 32768) { jumpnode_id = jps_id::none(); }
}

void
offline_jump_point_locator::jump_southwest(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	uint16_t label     = db_[8 * node_id.id + 7];
	uint16_t num_steps = label & 32767;
	uint32_t mapw      = map_->width();

	// goal test (so many div ops! and branches! how ugly!)
	if((goal_id.id - node_id.id) < map_->padded_mapsize())
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (gy - ny);
		uint32_t xdelta = (nx - gx);
		if(xdelta < mapw && ydelta < map_->height())
		{
			jumpnode_id = jps_id::none();
			uint32_t nid, steps_to_nid;
			if(ydelta < xdelta && ydelta <= num_steps)
			{
				steps_to_nid = ydelta;
				nid          = node_id.id + (mapw - 1) * steps_to_nid;
				jump_west(jps_id(nid), goal_id, jumpnode_id, jumpcost);
				if(jumpnode_id == goal_id)
				{
					jumpnode_id = goal_id;
					jumpcost = steps_to_nid * warthog::DBL_ROOT_TWO + jumpcost;
					return;
				}
			}
			else if(xdelta <= num_steps)
			{
				steps_to_nid = xdelta;
				nid          = node_id.id + (mapw - 1) * steps_to_nid;
				jump_south(jps_id(nid), goal_id, jumpnode_id, jumpcost);
				if(jumpnode_id == goal_id)
				{
					jumpnode_id = goal_id;
					jumpcost = steps_to_nid * warthog::DBL_ROOT_TWO + jumpcost;
					return;
				}
			}
		}
	}

	// return the jump point; but only if it isn't sterile
	jumpnode_id = jps_id(node_id.id + (mapw - 1) * num_steps);
	jumpcost    = num_steps * warthog::DBL_ROOT_TWO;
	if(label & 32768) { jumpnode_id = jps_id::none(); }
}

void
offline_jump_point_locator::jump_southeast(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	uint16_t label     = db_[8 * node_id.id + 6];
	uint16_t num_steps = label & 32767;
	uint32_t mapw      = map_->width();

	// goal test (so many div ops! and branches! how ugly!)
	if((goal_id.id - node_id.id) < map_->padded_mapsize())
	{
		uint32_t gx, gy, nx, ny;
		map_->to_padded_xy(goal_id, gx, gy);
		map_->to_padded_xy(node_id, nx, ny);

		uint32_t ydelta = (gy - ny);
		uint32_t xdelta = (gx - nx);
		if(xdelta < mapw && ydelta < map_->height())
		{
			uint32_t nid, steps_to_nid;
			jumpnode_id = jps_id::none();
			if(ydelta < xdelta && ydelta <= num_steps)
			{
				steps_to_nid = ydelta;
				nid          = node_id.id + (mapw + 1) * steps_to_nid;
				jump_east(jps_id(nid), goal_id, jumpnode_id, jumpcost);
				if(jumpnode_id == goal_id)
				{
					jumpnode_id = goal_id;
					jumpcost = steps_to_nid * warthog::DBL_ROOT_TWO + jumpcost;
					return;
				}
			}
			else if(xdelta <= num_steps)
			{
				steps_to_nid = xdelta;
				nid          = node_id.id + (mapw + 1) * steps_to_nid;
				jump_south(jps_id(nid), goal_id, jumpnode_id, jumpcost);
				if(jumpnode_id == goal_id)
				{
					jumpnode_id = goal_id;
					jumpcost = steps_to_nid * warthog::DBL_ROOT_TWO + jumpcost;
					return;
				}
			}
		}
	}

	jumpnode_id = jps_id(node_id.id + (mapw + 1) * num_steps);
	jumpcost    = num_steps * warthog::DBL_ROOT_TWO;
	if(label & 32768) { jumpnode_id = jps_id::none(); }
}

void
offline_jump_point_locator::jump_north(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
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
			jumpnode_id = goal_id;
			jumpcost    = (goal_delta / map_->width());
			return;
		}
	}

	// return the jump point at hand
	jumpnode_id = jps_id(node_id.id - id_delta);
	jumpcost    = num_steps;
	if(label & 32768) { jumpnode_id = jps_id::none(); }
}

void
offline_jump_point_locator::jump_south(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
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
			jumpnode_id = goal_id;
			jumpcost    = goal_delta / map_->width();
			return;
		}
	}

	// return the jump point at hand
	jumpnode_id = jps_id(node_id.id + id_delta);
	jumpcost    = num_steps;
	if(label & 32768) { jumpnode_id = jps_id::none(); }
}

void
offline_jump_point_locator::jump_east(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	uint16_t label = db_[8 * node_id.id + 2];

	// do not jump over the goal
	uint32_t id_delta   = label & 32767;
	uint32_t goal_delta = goal_id.id - node_id.id;
	if(id_delta >= goal_delta)
	{
		jumpnode_id = goal_id;
		jumpcost    = goal_delta;
		return;
	}

	// return the jump point at hand
	jumpnode_id = jps_id(node_id.id + id_delta);
	jumpcost    = id_delta;
	if(label & 32768) { jumpnode_id = jps_id::none(); }
}

void
offline_jump_point_locator::jump_west(
    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost)
{
	uint16_t label = db_[8 * node_id.id + 3];

	// do not jump over the goal
	uint32_t id_delta   = label & 32767;
	uint32_t goal_delta = node_id.id - goal_id.id;
	if(id_delta >= goal_delta)
	{
		jumpnode_id = goal_id;
		jumpcost    = goal_delta;
		return;
	}

	// return the jump point at hand
	jumpnode_id = jps_id(node_id.id - id_delta);
	jumpcost    = id_delta;
	if(label & 32768) { jumpnode_id = jps_id::none(); }
}

} // namespace jps::jump
