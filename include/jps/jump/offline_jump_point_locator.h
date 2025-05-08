#ifndef JPS_JUMP_OFFLINE_JUMP_POINT_LOCATOR_H
#define JPS_JUMP_OFFLINE_JUMP_POINT_LOCATOR_H

// offline_jump_point_locator.h
//
// Identifies jump points using a pre-computed database that stores
// distances from each node to jump points in every direction.
//
// @author: dharabor
// @created: 05/05/2013
//

#include <jps/forward.h>
#include <warthog/domain/gridmap.h>

namespace jps::jump
{

class offline_jump_point_locator
{
public:
	offline_jump_point_locator(warthog::domain::gridmap* map);
	~offline_jump_point_locator();

	void
	jump(
	    direction d, jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    double& jumpcost);

	uint32_t
	mem() const noexcept
	{
		return sizeof(this) + sizeof(*db_) * dbsize_;
	}

private:
	void
	preproc();

	bool
	load(const char* filename);

	void
	save(const char* filename);

	void
	jump_northwest(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost);
	void
	jump_northeast(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost);
	void
	jump_southwest(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost);
	void
	jump_southeast(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost);
	void
	jump_north(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost);
	void
	jump_south(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost);
	void
	jump_east(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost);
	void
	jump_west(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost);

	warthog::domain::gridmap* map_;
	uint32_t dbsize_;
	uint16_t* db_;

	// uint32_t jumppoints_[3];
	// double costs_[3];
	uint32_t max_;
	uint32_t current_;
};

}

#endif // JPS_JUMP_OFFLINE_JUMP_POINT_LOCATOR_H
