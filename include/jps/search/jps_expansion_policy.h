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
#include <warthog/search/gridmap_expansion_policy.h>
#include <jps/domain/rotate_gridmap.h>
#include <warthog/util/template.h>

namespace jps::search
{

/// @brief 
/// @tparam JpsJump 
///
/// JPS expansion policy that pushes the first cardinal and intercardinal
/// jump point, block-based jumping is the standard jump used by jump_point_online.
/// jps_2011_expansion_policy<jump_point_online> gives JPS (B).
/// jps_2011_expansion_policy<jump_point_offline> gives JPS+.
template<typename JpsJump>
class jps_expansion_policy
    : public warthog::search::gridmap_expansion_policy_base
{
public:
	jps_expansion_policy(warthog::domain::gridmap* map)
	    : gridmap_expansion_policy_base(map), rmap_(map)
	{
		jpl_.set_map(*map);
	}
	virtual ~jps_expansion_policy() = default;

	using jump_point = JpsJump;

	void
	expand(
	    warthog::search::search_node* current,
	    warthog::search::search_problem_instance* pi) override;

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
	domain::rotate_gridmap rmap_;
	JpsJump jpl_;
	point target_loc_;
	grid_id target_id_;
};

template<typename JpsJump>
void
jps_expansion_policy<JpsJump>::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* instance)
{
	reset();

	// compute the direction of travel used to reach the current node.
	const grid_id current_id = jps_id(current->get_id());
	const point loc = rmap_.id_to_point(current_id);
	// const jps_rid current_rid = jpl_.id_to_rid(current_id);
	// const cost_t current_cost = current->get_g();
	const direction dir_c = from_direction(
	    jps_id(current->get_parent()), current_id, map_->width());
	const direction_id target_d = warthog::grid::point_to_direction_id(loc, target_loc_);

	// get the tiles around the current node c
	uint32_t c_tiles;
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural
	// and forced neighbour
	uint32_t succ_dirs = compute_successors(dir_c, c_tiles);
	if (succ_dirs & static_cast<uint32_t>(warthog::grid::to_dir(target_d))) {
		// target in successor direction, check
		if (auto target_dist = jpl_.jump_target(loc, target_loc_); target_dist.second >= 0) {
			// target is visible, push
			warthog::search::search_node* jp_succ
				    = this->generate(target_id_);
			add_neighbour(jp_succ, target_dist.first * warthog::DBL_ROOT_TWO + target_dist.second * warthog::DBL_ONE);
			return; // no other successor required
		}
	}

	// cardinal directions
	::warthog::util::for_each_integer_sequence<
		std::integer_sequence<direction_id, NORTH_ID, EAST_ID, SOUTH_ID, WEST_ID>
	>([&]<direction_id di> {
		if(succ_dirs & warthog::grid::to_dir(di))
		{
			auto jump_result = jpl_.template jump_cardinal_next<di>(loc);
			if(jump_result > 0) // jump point
			{
				// successful jump
				warthog::search::search_node* jp_succ
				    = this->generate(pad_id(static_cast<int32_t>(current_id.id + warthog::grid::dir_id_adj(di) * jump_result)));
				add_neighbour(jp_succ, jump_result * warthog::DBL_ONE);
			}
		}
	});
	// intercardinal directions
	::warthog::util::for_each_integer_sequence<
		std::integer_sequence<direction_id, NORTHEAST_ID, NORTHWEST_ID, SOUTHEAST_ID, SOUTHWEST_ID>
	>([&]<direction_id di> {
		if(succ_dirs & warthog::grid::to_dir(di))
		{
			jump::intercardinal_jump_result res;
			auto jump_result = jpl_.template jump_intercardinal_many<di>(loc, &res, 1);
			if(jump_result.first > 0) // jump point
			{
				// successful jump
				warthog::search::search_node* jp_succ
				    = this->generate(pad_id(static_cast<int32_t>(current_id.id + warthog::grid::dir_id_adj(di) * jump_result.first)));
				add_neighbour(jp_succ, jump_result * warthog::DBL_ROOT_TWO);
			}
		}
	});
}

template<typename JpsJump>
warthog::search::search_node*
jps_expansion_policy<JpsJump>::generate_start_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->start_) >= max_id) { return nullptr; }
	pad_id padded_id = pad_id(pi->start_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	target_id_ = grid_id(pi->target_);
	target_loc_ = rmap_.map().to_unpadded_xy(target_id_);
	return generate(padded_id);
}

template<typename JpsJump>
warthog::search::search_node*
jps_expansion_policy<JpsJump>::generate_target_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->target_) >= max_id) { return nullptr; }
	pad_id padded_id = pad_id(pi->target_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

}

#endif // JPS_SEARCH_JPS_EXPANSION_POLICY2_H
