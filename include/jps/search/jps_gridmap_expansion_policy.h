#ifndef JPS_SEARCH_JPS_GRIDMAP_EXPANSION_POLICY_H
#define JPS_SEARCH_JPS_GRIDMAP_EXPANSION_POLICY_H

// jps_gridmap_expansion_policy.h
//
// This expansion policy is a base policy for the jps algorithms.
// It creates a common class for the rotated gridmap.

#include "jps.h"
#include <warthog/search/gridmap_expansion_policy.h>
#include <jps/domain/rotate_gridmap.h>

namespace jps::search
{

class jps_gridmap_expansion_policy
    : public warthog::search::gridmap_expansion_policy_base
{
public:
	jps_gridmap_expansion_policy(warthog::domain::gridmap* map)
	    : gridmap_expansion_policy_base(map)
	{
		if (map != nullptr) {
			rmap_.create_rmap(*map);
		}
	}

	size_t
	mem() override
	{
		return gridmap_expansion_policy_base::mem()
			+ (sizeof(jps_gridmap_expansion_policy) - sizeof(gridmap_expansion_policy_base))
			+ map_->mem();
	}

	void set_map(warthog::domain::gridmap& map)
	{
		rmap_.create_rmap(map);
		gridmap_expansion_policy_base::set_map(map);
		set_rmap_(rmap_);
	}
	void set_map(domain::gridmap_rotate_ptr rmap)
	{
		rmap_.link(rmap);
		gridmap_expansion_policy_base::set_map(rmap.map());
		set_rmap_(rmap_);
	}

protected:
	virtual void set_rmap_(domain::rotate_gridmap& rmap)
	{ }

protected:
	domain::rotate_gridmap rmap_;
};

} // namespace jps::search

#endif // JPS_SEARCH_JPS_GRIDMAP_EXPANSION_POLICY_H
