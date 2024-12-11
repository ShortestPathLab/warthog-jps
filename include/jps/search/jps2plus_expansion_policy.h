#ifndef JPS_SEARCH_JPS2PLUS_EXPANSION_POLICY_H
#define JPS_SEARCH_JPS2PLUS_EXPANSION_POLICY_H

// jps2plus_expansion_policy.h
//
// An experimental variation of warthog::jps_expansion_policy,
// this version is designed for efficient offline jps.
//
// @author: dharabor
// @created: 06/01/2010

#include "jps.h"
#include <jps/jump/offline_jump_point_locator2.h>
#include <warthog/domain/gridmap.h>
#include <warthog/search/gridmap_expansion_policy.h>
#include <warthog/search/problem_instance.h>
#include <warthog/search/search_node.h>
#include <warthog/util/helpers.h>

#include <cstdint>

namespace jps::search
{

class jps2plus_expansion_policy
    : public warthog::search::gridmap_expansion_policy_base
{
public:
	jps2plus_expansion_policy(warthog::domain::gridmap* map);
	virtual ~jps2plus_expansion_policy();

	void
	expand(
	    warthog::search::search_node*,
	    warthog::search::search_problem_instance*) override;

	size_t
	mem() override
	{
		return expansion_policy::mem() + sizeof(*this) + map_->mem()
		    + jpl_->mem();
	}

	warthog::search::search_node*
	generate_start_node(warthog::search::search_problem_instance* pi) override;

	warthog::search::search_node*
	generate_target_node(
	    warthog::search::search_problem_instance* pi) override;

private:
	jump::offline_jump_point_locator2* jpl_;
	vec_jps_cost costs_;
	vec_jps_id jp_ids_;
};

}

#endif // JPS_SEARCH_JPS2PLUS_EXPANSION_POLICY_H
