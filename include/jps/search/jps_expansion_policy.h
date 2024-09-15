#ifndef JPS_SEARCH_JPS_EXPANSION_POLICY_H
#define JPS_SEARCH_JPS_EXPANSION_POLICY_H

// jps_expansion_policy.h
//
// This expansion policy reduces the branching factor
// of a node n during search by ignoring any neighbours which
// could be reached by an equivalent (or shorter) path that visits
// the parent of n but not n itself.
//
// An extension of this idea is to generate jump nodes located in the
// same direction as the remaining neighbours.
//
// Theoretical details:
// [Harabor D. and Grastien A., 2011, Online Node Pruning for Pathfinding
// On Grid Maps, AAAI]
//
// @author: dharabor
// @created: 06/01/2010

#include "jps.h"
#include <cstdint>
#include <jps/jump/online_jump_point_locator.h>
#include <warthog/domain/gridmap.h>
#include <warthog/search/expansion_policy.h>
#include <warthog/search/problem_instance.h>
#include <warthog/search/search_node.h>
#include <warthog/util/helpers.h>

namespace jps::search
{

class jps_expansion_policy : public warthog::search::expansion_policy
{
public:
	jps_expansion_policy(warthog::domain::gridmap* map);
	virtual ~jps_expansion_policy();

	virtual void
	expand(warthog::search::search_node*, warthog::search::problem_instance*);

	uint32_t
	get_state(warthog::sn_id_t node_id);

	void
	print_node(warthog::search::search_node* n, std::ostream& out);

	virtual warthog::search::search_node*
	generate_start_node(warthog::search::problem_instance* pi);

	virtual warthog::search::search_node*
	generate_target_node(warthog::search::problem_instance* pi);

	size_t
	mem() override
	{
		return expansion_policy::mem() + sizeof(*this) + map_->mem()
		    + jpl_->mem();
	}

private:
	warthog::domain::gridmap* map_;
	warthog::online_jump_point_locator* jpl_;
};

}

#endif // JPS_SEARCH_JPS_EXPANSION_POLICY_H
