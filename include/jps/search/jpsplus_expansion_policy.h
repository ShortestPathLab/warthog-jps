#ifndef JPS_SEARCH_JPSPLUS_EXPANSION_POLICY_H
#define JPS_SEARCH_JPSPLUS_EXPANSION_POLICY_H

// jpsplus_expansion_policy.h
//
// JPS+ is Jump Point Search together with a preprocessed database
// that stores all jump points for every node.
//
// Theoretical details:
// [Harabor and Grastien, 2012, The JPS Pathfinding System, SoCS]
//
// @author: dharabor
// @created: 05/05/2012

#include "jps.h"
#include <jps/jump/offline_jump_point_locator.h>
#include <warthog/domain/gridmap.h>
#include <warthog/search/gridmap_expansion_policy.h>
#include <warthog/search/problem_instance.h>
#include <warthog/search/search_node.h>
#include <warthog/util/helpers.h>

#include <cstdint>

namespace jps::search
{

class jpsplus_expansion_policy
    : public warthog::search::gridmap_expansion_policy_base
{
public:
	jpsplus_expansion_policy(warthog::domain::gridmap* map);
	virtual ~jpsplus_expansion_policy();

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
	jump::offline_jump_point_locator* jpl_;
};

}

#endif // JPS_SEARCH_JPSPLUS_EXPANSION_POLICY_H
