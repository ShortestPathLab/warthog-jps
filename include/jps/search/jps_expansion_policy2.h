#ifndef JPS_SEARCH_JPS_EXPANSION_POLICY2_H
#define JPS_SEARCH_JPS_EXPANSION_POLICY2_H

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
#include <jps/jump/jump_point_online.h>
#include <warthog/search/gridmap_expansion_policy.h>

namespace jps::search
{

template<typename JpsJump>
class jps_expansion_policy2
    : public warthog::search::gridmap_expansion_policy_base
{
public:
	jps_expansion_policy2(warthog::domain::gridmap* map, bool setup_map = true)
	    : gridmap_expansion_policy_base(map), jpl_(setup_map ? map : nullptr)
	{ }
	virtual ~jps_expansion_policy2() = default;

	using jump_point = JpsJump;

	static consteval bool
	feature_prune_intercardinal() noexcept
	{
		return jump_point::feature_prune_intercardinal();
	}
	static consteval bool
	feature_store_cardinal() noexcept
	{
		return jump_point::feature_store_cardinal();
	}

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
		    + jpl_.mem();
	}

	jump_point&
	get_jump_point() noexcept
	{
		return jpl_;
	}
	const jump_point&
	get_jump_point() const noexcept
	{
		return jpl_;
	}

private:
	JpsJump jpl_;
};

template<typename JpsJump>
void
jps_expansion_policy2<JpsJump>::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* instance)
{
	reset();

	// compute the direction of travel used to reach the current node.
	const jps_id current_id   = jps_id(current->get_id());
	const jps_rid current_rid = jpl_.id_to_rid(current_id);
	// const cost_t current_cost = current->get_g();
	const direction dir_c = from_direction(
	    jps_id(current->get_parent()), current_id, map_->width());

	// get the tiles around the current node c
	uint32_t c_tiles;
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural
	// and forced neighbour
	uint32_t succ_dirs = compute_successors(dir_c, c_tiles);

	// cardinal directions
	for(uint32_t i = 0; i < 4; i++)
	{
		direction d = (direction)(1 << i);
		if(succ_dirs & d)
		{
			auto jump_result = jpl_.jump_cardinal(d, current_id, current_rid);
			if(jump_result.first > 0) // jump point
			{
				// successful jump
				warthog::search::search_node* jp_succ
				    = this->generate(jump_result.second);
				// if(jp_succ->get_searchid() != search_id) {
				// jp_succ->reset(search_id); }
				add_neighbour(jp_succ, jump_result.first * warthog::DBL_ONE);
			}
		}
	}
	// intercardinal
	constexpr size_t buffer_size = feature_prune_intercardinal() ? 1024
	    : feature_store_cardinal()                               ? 4
	                                                             : 1;
	std::array<jps_id, buffer_size> id_array [[maybe_unused]];
	std::array<cost_t, buffer_size> cost_array [[maybe_unused]];
	for(uint32_t i = 4; i < 8; i++)
	{
		direction d = (direction)(1 << i);
		if(succ_dirs & d)
		{
			jump::intercardinal_jump_result jump_result{
			    current_id, current_rid, 0};
			uint32_t jump_span [[maybe_unused]] = 0;
			while(true)
			{
				// exit conditions: do while
				//    => feature_prune_intercardinal(): jump_span.dist != 0 #
				//    if buffer_size is not large enough, do multiple
				//    iterations
				//    => otherwise: always break, only single run required
				if constexpr(
				    feature_prune_intercardinal() || feature_store_cardinal())
				{
					jump_result = jpl_.jump_intercardinal(
					    d, jump_result.node, jump_result.rnode,
					    id_array.data(), cost_array.data(), id_array.size());
				}
				else
				{
					jump_result = jpl_.jump_intercardinal(
					    d, jump_result.node, jump_result.rnode, nullptr,
					    nullptr);
				}
				// add successor from id_array (if any)
				if constexpr(feature_prune_intercardinal())
				{
					if(jump_result.dist != 0)
					{
						// successful jump
						for(uint32_t j = 0; j < jump_result.dist; ++j)
						{
							warthog::search::search_node* jp_succ
							    = this->generate(id_array[j]);
							// if(jp_succ->get_searchid() != search_id) {
							// jp_succ->reset(search_id); }
							add_neighbour(
							    jp_succ,
							    jump_span * warthog::DBL_ROOT_TWO
							        + cost_array[j]);
						}
					}
				}
				else if constexpr(feature_store_cardinal())
				{
					for(uint32_t j = 0; j < 2; ++j)
					{
						if(auto expand_id = id_array[j]; !expand_id.is_none())
						{
							warthog::search::search_node* jp_succ
							    = this->generate(expand_id);
							// if(jp_succ->get_searchid() != search_id) {
							// jp_succ->reset(search_id); }
							add_neighbour(jp_succ, cost_array[j]);
						}
					}
				}
				// add successor return from function
				if constexpr(!feature_prune_intercardinal())
				{
					if(jump_result.dist != 0)
					{
						warthog::search::search_node* jp_succ
						    = this->generate(jump_result.node);
						// if(jp_succ->get_searchid() != search_id) {
						// jp_succ->reset(search_id); }
						add_neighbour(
						    jp_succ, jump_result.dist * warthog::DBL_ROOT_TWO);
					}
					break; // do not loop
				}
				else
				{
					if(jump_result.node.is_none())
						break; // exit loop after reaching end
				}
			}
		}
	}
}

template<typename JpsJump>
warthog::search::search_node*
jps_expansion_policy2<JpsJump>::generate_start_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->start_) >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(pi->start_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	jpl_.set_goal(jps_id(pi->target_));
	return generate(padded_id);
}

template<typename JpsJump>
warthog::search::search_node*
jps_expansion_policy2<JpsJump>::generate_target_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->target_) >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(pi->target_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

}

#endif // JPS_SEARCH_JPS_EXPANSION_POLICY2_H
