#ifndef JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR_H
#define JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR_H

// online_jump_point_locator.h
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
#include <warthog/domain/gridmap.h>

namespace jps::jump
{

class online_jump_point_locator
{
public:
	online_jump_point_locator(warthog::domain::gridmap* map);
	~online_jump_point_locator();

	void
	jump(
	    direction d, jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);

	size_t
	mem() const noexcept
	{
		return sizeof(this) + rmap_->mem();
	}

private:
	void
	jump_northwest(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);
	void
	jump_northeast(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);
	void
	jump_southwest(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);
	void
	jump_southeast(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);
	void
	jump_north(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);
	void
	jump_south(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);
	void
	jump_east(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);
	void
	jump_west(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost);

	// these versions can be passed a map parameter to
	// use when jumping. they allow switching between
	// map_ and rmap_ (a rotated counterpart).
	void
	jump_east_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);
	void
	jump_west_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);
	void
	jump_north_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);
	void
	jump_south_(
	    jps_id node_id, jps_id goal_id, jps_id& jumpnode_id,
	    warthog::cost_t& jumpcost, warthog::domain::gridmap* mymap);

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
};

}

#endif // JPS_JUMP_ONLINE_JUMP_POINT_LOCATOR_H
