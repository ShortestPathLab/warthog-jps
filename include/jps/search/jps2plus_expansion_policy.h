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
#include <warthog/search/expansion_policy.h>
#include <warthog/search/problem_instance.h>
#include <warthog/search/search_node.h>
#include <warthog/util/helpers.h>

#include <cstdint>

namespace jps::search
{

class jps2plus_expansion_policy : public warthog::search::expansion_policy
{
public:
	jps2plus_expansion_policy(warthog::domain::gridmap* map);
	virtual ~jps2plus_expansion_policy();

	virtual void
	expand(warthog::search::search_node*, warthog::search::problem_instance*);

	virtual size_t
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
	offline_jump_point_locator2* jpl_;
	std::vector<double> costs_;
	vec_jps_id jp_ids_;
};

}

#endif // JPS_SEARCH_JPS2PLUS_EXPANSION_POLICY_H
