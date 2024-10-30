#ifndef JPS_JUMP_FOUR_CONNECTED_JPS_LOCATOR_H
#define JPS_JUMP_FOUR_CONNECTED_JPS_LOCATOR_H

// jps/four_connected_jps_locator.h
//
// Implements grid scanning operations for
// Jump Point Search in 4-connected gridmaps.
//
// NB: based on the class warthog::online_jump_point_locator
//
// @author: dharabor
// @created: 2019-11-13
//

#include <jps/forward.h>

namespace jps::jump
{

class four_connected_jps_locator
{
public:
	four_connected_jps_locator(warthog::gridmap* map);
	~four_connected_jps_locator();

	void
	jump(
	    direction d, uint32_t node_id, uint32_t goalid, uint32_t& jumpnode_id,
	    double& jumpcost);

	size_t
	mem()
	{
		return sizeof(this);
	}

	// private:
	void
	jump_north(
	    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
	    double& jumpcost);
	void
	jump_south(
	    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
	    double& jumpcost);
	void
	jump_east(
	    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
	    double& jumpcost);
	void
	jump_west(
	    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
	    double& jumpcost);

	// these versions can be passed a map parameter to
	// use when jumping. they allow switching between
	// map_ and rmap_ (a rotated counterpart).
	void
	jump_east_(
	    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
	    double& jumpcost, warthog::gridmap* mymap);
	void
	jump_west_(
	    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
	    double& jumpcost, warthog::gridmap* mymap);
	void
	jump_north_(
	    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
	    double& jumpcost, warthog::gridmap* mymap);
	void
	jump_south_(
	    uint32_t node_id, uint32_t goal_id, uint32_t& jumpnode_id,
	    double& jumpcost, warthog::gridmap* mymap);

	warthog::gridmap* map_;
	// uint32_t jumplimit_;
};

} // namespace jps::jump

#endif // JPS_JUMP_FOUR_CONNECTED_JPS_LOCATOR_H
