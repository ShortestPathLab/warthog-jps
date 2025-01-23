#ifndef JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR2_H
#define JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR2_H

// online_jump_point_locator2.h
//
// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011,
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//
// @author: dharabor
// @created: 03/09/2012
//

#include <jps/forward.h>
#include <vector>
#include <warthog/domain/gridmap.h>

namespace jps::jump
{

class online_jump_point_locator2
{
public:
	online_jump_point_locator2(warthog::domain::gridmap* map);
	~online_jump_point_locator2();

	void
	jump(
	    direction d, jps_id node_id, jps_id goal_id, vec_jps_id& jpoints,
	    vec_jps_cost& costs);

	size_t
	mem() const noexcept
	{
		return sizeof(this) + rmap_->mem();
	}

private:
	void
	jump_north(vec_jps_id& jpoints, vec_jps_cost& costs);
	void
	jump_south(vec_jps_id& jpoints, vec_jps_cost& costs);
	void
	jump_east(vec_jps_id& jpoints, vec_jps_cost& costs);
	void
	jump_west(vec_jps_id& jpoints, vec_jps_cost& costs);
	void
	jump_northeast(vec_jps_id& jpoints, vec_jps_cost& costs);
	void
	jump_northwest(vec_jps_id& jpoints, vec_jps_cost& costs);
	void
	jump_southeast(vec_jps_id& jpoints, vec_jps_cost& costs);
	void
	jump_southwest(vec_jps_id& jpoints, vec_jps_cost& costs);

	// these versions can be passed a map parameter to
	// use when jumping. they allow switching between
	// map_ and rmap_ (a rotated counterpart).
	void
	jump_north_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);
	void
	jump_south_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);
	void
	jump_east_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);
	void
	jump_west_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);

	// these versions perform a single diagonal jump, returning
	// the intermediate diagonal jump point and the straight
	// jump points that caused the jumping process to stop
	void
	jump_northeast_(
	    jps_id& node_id, jps_id& rnode_id, jps_id goal_id, jps_id rgoal_id,
	    jps_id& jumpnode_id, warthog::cost_t& jumpcost, jps_id& jp1_id,
	    warthog::cost_t& jp1_cost, jps_id& jp2_id, warthog::cost_t& jp2_cost);
	void
	jump_northwest_(
	    jps_id& node_id, jps_id& rnode_id, jps_id goal_id, jps_id rgoal_id,
	    jps_id& jumpnode_id, warthog::cost_t& jumpcost, jps_id& jp1_id,
	    warthog::cost_t& jp1_cost, jps_id& jp2_id, warthog::cost_t& jp2_cost);
	void
	jump_southeast_(
	    jps_id& node_id, jps_id& rnode_id, jps_id goal_id, jps_id rgoal_id,
	    jps_id& jumpnode_id, warthog::cost_t& jumpcost, jps_id& jp1_id,
	    warthog::cost_t& jp1_cost, jps_id& jp2_id, warthog::cost_t& jp2_cost);
	void
	jump_southwest_(
	    jps_id& node_id, jps_id& rnode_id, jps_id goal_id, jps_id rgoal_id,
	    jps_id& jumpnode_id, warthog::cost_t& jumpcost, jps_id& jp1_id,
	    warthog::cost_t& jp1_cost, jps_id& jp2_id, warthog::cost_t& jp2_cost);

	// functions to convert map indexes to rmap indexes
	inline jps_id
	map_id_to_rmap_id(jps_id mapid)
	{
		if(mapid.is_none()) { return jps_id::none(); }

		uint32_t x, y;
		uint32_t rx, ry;
		map_->to_unpadded_xy(mapid, x, y);
		ry = x;
		rx = map_->header_height() - y - 1;
		return jps_id(rmap_->to_padded_id_from_unpadded(rx, ry));
	}

	// convert rmap indexes to map indexes
	inline jps_id
	rmap_id_to_map_id(jps_id rmapid)
	{
		if(rmapid.is_none()) { return jps_id::none(); }

		uint32_t x, y;
		uint32_t rx, ry;
		rmap_->to_unpadded_xy(rmapid, rx, ry);
		x = ry;
		y = rmap_->header_width() - rx - 1;
		return jps_id(map_->to_padded_id_from_unpadded(x, y));
	}

	warthog::domain::gridmap*
	create_rmap();

	warthog::domain::gridmap* map_;
	warthog::domain::gridmap* rmap_;
	// uint32_t jumplimit_;

	jps_id current_goal_id_;
	jps_id current_rgoal_id_;
	jps_id current_node_id_;
	jps_id current_rnode_id_;

	// these function pointers allow us to switch between forward jumping
	// and backward jumping (i.e. with the parent direction reversed)
	void (online_jump_point_locator2::*jump_east_fp)(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);

	void (online_jump_point_locator2::*jump_west_fp)(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);
};

} // namespace jps::jump

#endif // JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR2_H
