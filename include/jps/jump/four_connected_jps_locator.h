#ifndef JPS_JUMP_FOUR_CONNECTED_JPS_LOCATOR_H
#define JPS_JUMP_FOUR_CONNECTED_JPS_LOCATOR_H

// jps/four_connected_jps_locator.h
//
// Implements grid scanning operations for
// Jump Point Search in 4-connected gridmaps.
//
// NB: based on the class online_jump_point_locator
//
// @author: dharabor
// @created: 2019-11-13
//

#include <jps/forward.h>
#include <warthog/domain/gridmap.h>

namespace jps::jump
{

class four_connected_jps_locator
{
public:
	four_connected_jps_locator(warthog::domain::gridmap* map);
	~four_connected_jps_locator();

	void
	jump(
	    direction d, jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    double& jumpcost);

	size_t
	mem() const noexcept
	{
		return sizeof(this);
	}

	// private:
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

	// these versions can be passed a map parameter to
	// use when jumping. they allow switching between
	// map_ and rmap_ (a rotated counterpart).
	void
	jump_east_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost,
	    warthog::domain::gridmap* mymap);
	void
	jump_west_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost,
	    warthog::domain::gridmap* mymap);
	void
	jump_north_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost,
	    warthog::domain::gridmap* mymap);
	void
	jump_south_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id, double& jumpcost,
	    warthog::domain::gridmap* mymap);

	warthog::domain::gridmap* map_;
	// uint32_t jumplimit_;
};

} // namespace jps::jump

#endif // JPS_JUMP_FOUR_CONNECTED_JPS_LOCATOR_H
