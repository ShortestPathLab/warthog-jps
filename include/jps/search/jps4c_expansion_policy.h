#ifndef JPS_SEARCH_JPS4C_EXPANSION_POLICY_H
#define JPS_SEARCH_JPS4C_EXPANSION_POLICY_H

// jps/jps4c_expansion_policy.h
//
// Successor generating functions for Jump Point Search on 4-connected gridmaps
//
// @author: dharabor
// @created: 2019-11-13
//

#include "jps.h"
#include <jps/jump/four_connected_jps_locator.h>
#include <jps/jump/online_jump_point_locator.h>
#include <warthog/domain/gridmap.h>
#include <warthog/search/gridmap_expansion_policy.h>
#include <warthog/search/problem_instance.h>
#include <warthog/search/search_node.h>
#include <warthog/util/helpers.h>

#include <cstdint>

namespace jps::search
{

class jps4c_expansion_policy
    : public warthog::search::gridmap_expansion_policy_base
{
public:
	jps4c_expansion_policy(warthog::domain::gridmap* map);
	virtual ~jps4c_expansion_policy();

	void
	expand(
	    warthog::search::search_node*,
	    warthog::search::search_problem_instance*) override;

	warthog::search::search_node*
	generate_start_node(warthog::search::search_problem_instance* pi) override;

	warthog::search::search_node*
	generate_target_node(
	    warthog::search::search_problem_instance* pi) override;

	size_t
	mem() override
	{
		return expansion_policy::mem() + sizeof(*this) + map_->mem()
		    + jpl_->mem();
	}

private:
	jump::four_connected_jps_locator* jpl_;
};

}

#endif // JPS_SEARCH_JPS4C_EXPANSION_POLICY_H
