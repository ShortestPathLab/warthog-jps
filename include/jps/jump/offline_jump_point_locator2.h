#ifndef JPS_JUMP_OFFLINE_JUMP_POINT_LOCATOR2_H
#define JPS_JUMP_OFFLINE_JUMP_POINT_LOCATOR2_H

// offline_jump_point_locator2.h
//
// Variant of warthog::offline_jump_point_locator.
// Jump points are identified using a pre-computed database that stores
// distances from each node to jump points in every direction.
// This version additionally prunes all jump points that do not have at
// least one forced neighbour.
//
// @author: dharabor
// @created: 05/05/2013
//

#include <jps/forward.h>
#include <vector>
#include <warthog/domain/gridmap.h>

namespace jps::jump
{

class offline_jump_point_locator2
{
public:
	offline_jump_point_locator2(warthog::domain::gridmap* map);
	~offline_jump_point_locator2();

	void
	jump(
	    direction d, jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
	    vec_jps_cost& costs);

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
	    jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
	    vec_jps_cost& costs);
	void
	jump_northeast(
	    jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
	    vec_jps_cost& costs);
	void
	jump_southwest(
	    jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
	    vec_jps_cost& costs);
	void
	jump_southeast(
	    jps_id node_id, jps_id goal_id, vec_jps_id& neighbours,
	    vec_jps_cost& costs);
	void
	jump_north(
	    jps_id node_id, jps_id goal_id, double cost_to_node_id,
	    vec_jps_id& neighbours, vec_jps_cost& costs);
	void
	jump_south(
	    jps_id node_id, jps_id goal_id, double cost_to_node_id,
	    vec_jps_id& neighbours, vec_jps_cost& costs);
	void
	jump_east(
	    jps_id node_id, jps_id goal_id, double cost_to_node_id,
	    vec_jps_id& neighbours, vec_jps_cost& costs);
	void
	jump_west(
	    jps_id node_id, jps_id goal_id, double cost_to_node_id,
	    vec_jps_id& neighbours, vec_jps_cost& costs);

	warthog::domain::gridmap* map_;
	uint32_t dbsize_;
	uint16_t* db_;
};

}

#endif // JPS_JUMP_OFFLINE_JUMP_POINT_LOCATOR2_H
