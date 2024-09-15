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
#include <warthog/search/expansion_policy.h>
#include <warthog/search/problem_instance.h>
#include <warthog/search/search_node.h>
#include <warthog/util/helpers.h>

#include <cstdint>

namespace jps::search
{

class jpsplus_expansion_policy : public warthog::search::expansion_policy
{
public:
	jpsplus_expansion_policy(warthog::domain::gridmap* map);
	virtual ~jpsplus_expansion_policy();

	virtual void
	expand(warthog::search::search_node*, warthog::search::problem_instance*);

	virtual inline size_t
	mem()
	{
		return expansion_policy::mem() + sizeof(*this) + map_->mem()
		    + jpl_->mem();
	}

	uint32_t
	get_state(warthog::sn_id_t node_id);

	void
	print_node(warthog::search::search_node* n, std::ostream& out);

	virtual warthog::search::search_node*
	generate_start_node(warthog::search::problem_instance* pi);

	virtual warthog::search::search_node*
	generate_target_node(warthog::search::problem_instance* pi);

private:
	warthog::domain::gridmap* map_;
	offline_jump_point_locator* jpl_;
};

}

#endif // JPS_SEARCH_JPSPLUS_EXPANSION_POLICY_H
