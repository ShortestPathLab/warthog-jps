#ifndef JPS_SEARCH_JPS2_EXPANSION_POLICY_H
#define JPS_SEARCH_JPS2_EXPANSION_POLICY_H

// jps2_expansion_policy.h
//
// An experimental variation of warthog::jps2_expansion_policy,
// this version works with a modified version of online JPS
// which skips intermediate jump points (i.e. those jps
// that do not have any forced neighbours)
//
// @author: dharabor
// @created: 06/01/2010

#include "jps.h"
#include <jps/jump/online_jump_point_locator2.h>
#include <warthog/domain/gridmap.h>
#include <warthog/search/expansion_policy.h>
#include <warthog/search/problem_instance.h>
#include <warthog/search/search_node.h>
#include <warthog/util/helpers.h>

namespace jps::search
{

class jps2_expansion_policy : public warthog::search::expansion_policy
{
public:
	jps2_expansion_policy(warthog::domain::gridmap* map);
	virtual ~jps2_expansion_policy();

	void
	expand(warthog::search::search_node*, warthog::search::problem_instance*)
	    override;

	size_t
	mem() override
	{
		return expansion_policy::mem() + sizeof(*this) + map_->mem()
		    + jpl_->mem();
	}

	uint32_t
	get_state(warthog::sn_id_t node_id);

	void
	print_node(warthog::search::search_node* n, std::ostream& out);

	warthog::search::search_node*
	generate_start_node(warthog::search::problem_instance* pi) override;

	warthog::search::search_node*
	generate_target_node(warthog::search::problem_instance* pi) override;

	// this function gets called whenever a successor node is relaxed. at that
	// point we set the node currently being expanded (==current) as the
	// parent of n and label node n with the direction of travel,
	// from current to n
	// void
	// update_parent_direction(warthog::search::search_node* n);

private:
	warthog::domain::gridmap* map_;
	warthog::jps::online_jump_point_locator2* jpl_;
	std::vector<uint32_t> jp_ids_;
	std::vector<warthog::cost_t> jp_costs_;
};

}

#endif // JPS_SEARCH_JPS2_EXPANSION_POLICY_H
