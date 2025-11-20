#ifndef JPS_SEARCH_JPS_EXPANSION_POLICY_BASE_H
#define JPS_SEARCH_JPS_EXPANSION_POLICY_BASE_H

//
// jps_expansion_policy_base.h
//
// The base expansion policy for jps-base algorithm.
// Adds interface for setting the domain, rotate_gridmap.
//
//
// @author: Ryan Hechenberger
// @created: 06/01/2010
//

#include "jps.h"
#include <jps/domain/rotate_gridmap.h>
#include <warthog/search/gridmap_expansion_policy.h>

namespace jps::search
{

class jps_expansion_policy_base
    : public warthog::search::gridmap_expansion_policy_base
{
public:
	/// @brief sets the policy to use with map
	/// @param map point to gridmap, if null map will need to be set later;
	///            otherwise sets map and creates a rotated gridmap.
	///            Use set_map to provide a map at a later stage.
	jps_expansion_policy_base(warthog::domain::gridmap* map)
	    : gridmap_expansion_policy_base(map)
	{
		if(map != nullptr) { rmap_.create_rmap(*map); }
	}

	size_t
	mem() override
	{
		return gridmap_expansion_policy_base::mem()
		    + (sizeof(jps_expansion_policy_base)
		       - sizeof(gridmap_expansion_policy_base))
		    + rmap_.mem();
	}

	/// @brief Sets the map, creating the rotated gridmap from map at current
	/// state.
	/// @param map the gridmap to use and rotate
	void
	set_map(warthog::domain::gridmap& map)
	{
		rmap_.create_rmap(map);
		gridmap_expansion_policy_base::set_map(map);
		set_rmap_(rmap_);
	}
	/// @brief Gives a user-proved map and rotated map, the expander does not
	/// own either resource.
	/// @param rmap the rotated_gridmap pair-point struct
	void
	set_map(domain::gridmap_rotate_ptr rmap)
	{
		rmap_.link(rmap);
		gridmap_expansion_policy_base::set_map(rmap.map());
		set_rmap_(rmap_);
	}

protected:
	virtual void
	set_rmap_(domain::rotate_gridmap& rmap)
	{ }

protected:
	domain::rotate_gridmap rmap_;
};

} // namespace jps::search

#endif // JPS_SEARCH_JPS_EXPANSION_POLICY_BASE_H
