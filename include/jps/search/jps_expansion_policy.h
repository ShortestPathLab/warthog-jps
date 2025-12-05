#ifndef JPS_SEARCH_JPS_EXPANSION_POLICY_H
#define JPS_SEARCH_JPS_EXPANSION_POLICY_H

//
// jps_expansion_policy.h
//
// This expansion policy reduces the branching factor
// of a node n during search by ignoring any neighbours which
// could be reached by an equivalent (or shorter) path that visits
// the parent of n but not n itself.
//
// Used with jump_point_online gives JPS (B) algorithm.
// Used with jump_point_offline gives JPS+ (B) algorithm.
//
// Theoretical details:
// [Harabor D. and Grastien A., 2011, Online Node Pruning for Pathfinding
// On Grid Maps, AAAI]
//
// @author: dharabor & Ryan Hechenberger
// @created: 06/01/2010
//

#include "jps_expansion_policy_base.h"
#include <jps/domain/rotate_gridmap.h>
#include <warthog/search/gridmap_expansion_policy.h>
#include <warthog/util/template.h>

namespace jps::search
{

/// @brief generates successor nodes for use in warthog-core search algorithm
/// @tparam JpsJump the jump-point locator to use
///
/// JPS expansion policy that pushes the first cardinal and first intercardinal
/// jump points for all taut directions.
/// The given JpsJump expects a class from jps::jump namespace,
/// such as jump_point_online for online JPS (B), or jump_point_offline<> for
/// offline JPS+ (B).
template<typename JpsJump>
class jps_expansion_policy : public jps_expansion_policy_base
{
public:
	/// @brief sets the policy to use with map
	/// @param map point to gridmap, if null map will need to be set later;
	///            otherwise sets map and creates a rotated gridmap.
	///            Use set_map to provide a map at a later stage.
	jps_expansion_policy(warthog::domain::gridmap* map)
	    : jps_expansion_policy_base(map)
	{
		if(map != nullptr)
		{
			jpl_.set_map(rmap_);
			map_width_ = rmap_.map().width();
		}
	}
	~jps_expansion_policy() = default;

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
		return jps_expansion_policy_base::mem()
		    + (sizeof(jps_expansion_policy)
		       - sizeof(jps_expansion_policy_base));
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

protected:
	void
	set_rmap_(domain::rotate_gridmap& rmap) override
	{
		jps_expansion_policy_base::set_rmap_(rmap);
		jpl_.set_map(rmap);
		map_width_ = rmap.map().width();
	}

private:
	JpsJump jpl_;
	point target_loc_   = {};
	grid_id target_id_  = {};
	uint32_t map_width_ = 0;
};

template<typename JpsJump>
void
jps_expansion_policy<JpsJump>::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* instance)
{
	reset();

	// compute the direction of travel used to reach the current node.
	const grid_id current_id = grid_id(current->get_id());
	const point loc          = rmap_.id_to_point(current_id);
	const domain::grid_pair_id pair_id{
	    current_id, rmap_.rpoint_to_rid(rmap_.point_to_rpoint(loc))};
	assert(
	    rmap_.map().get_label(get<grid_id>(pair_id))
	    && rmap_.rmap().get_label(
	        grid_id(get<rgrid_id>(pair_id)))); // loc must be trav on map
	// const jps_rid current_rid = jpl_.id_to_rid(current_id);
	// const cost_t current_cost = current->get_g();
	const direction dir_c = from_direction(
	    grid_id(current->get_parent()), current_id, rmap_.map().width());
	const direction_id target_d
	    = warthog::grid::point_to_direction_id(loc, target_loc_);

	// get the tiles around the current node c
	uint32_t c_tiles;
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural
	// and forced neighbour
	uint32_t succ_dirs = compute_successors(dir_c, c_tiles);
	if(succ_dirs & static_cast<uint32_t>(warthog::grid::to_dir(target_d)))
	{
		// target in successor direction, check
		if(auto target_dist = jpl_.jump_target(pair_id, loc, target_loc_);
		   target_dist.second >= 0)
		{
			// target is visible, push
			warthog::search::search_node* jp_succ = this->generate(target_id_);
			add_neighbour(
			    jp_succ,
			    target_dist.first * warthog::DBL_ROOT_TWO
			        + target_dist.second * warthog::DBL_ONE);
			return; // no other successor required
		}
	}

	// cardinal directions
	::warthog::util::for_each_integer_sequence<std::integer_sequence<
	    direction_id, NORTH_ID, EAST_ID, SOUTH_ID, WEST_ID>>([&](auto iv) {
		constexpr direction_id di = decltype(iv)::value;
		if(succ_dirs & warthog::grid::to_dir(di))
		{
			auto jump_result = jpl_.template jump_cardinal_next<di>(pair_id);
			if(jump_result > 0) // jump point
			{
				// successful jump
				pad_id node{static_cast<uint32_t>(
				    current_id.id
				    + warthog::grid::dir_id_adj(di, map_width_)
				        * jump_result)};
				assert(rmap_.map().get(node)); // successor must be traversable
				warthog::search::search_node* jp_succ = this->generate(node);
				add_neighbour(jp_succ, jump_result * warthog::DBL_ONE);
			}
		}
	});
	// intercardinal directions
	::warthog::util::for_each_integer_sequence<std::integer_sequence<
	    direction_id, NORTHEAST_ID, NORTHWEST_ID, SOUTHEAST_ID, SOUTHWEST_ID>>(
	    [&](auto iv) {
		    constexpr direction_id di = decltype(iv)::value;
		    if(succ_dirs & warthog::grid::to_dir(di))
		    {
			    jump::intercardinal_jump_result res;
			    auto jump_result = jpl_.template jump_intercardinal_many<di>(
			        pair_id, &res, 1);
			    if(jump_result.first > 0) // jump point
			    {
				    // successful jump
				    pad_id node{pad_id(static_cast<int32_t>(
				        current_id.id
				        + warthog::grid::dir_id_adj(di, map_width_)
				            * res.inter))};
				    assert(rmap_.map().get(
				        node)); // successor must be traversable
				    warthog::search::search_node* jp_succ
				        = this->generate(node);
				    add_neighbour(jp_succ, res.inter * warthog::DBL_ROOT_TWO);
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
	uint32_t x, y;
	rmap_.map().to_padded_xy(target_id_, x, y);
	target_loc_.x = static_cast<uint16_t>(x);
	target_loc_.y = static_cast<uint16_t>(y);
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

} // namespace jps::search

#endif // JPS_SEARCH_JPS_EXPANSION_POLICY_H
