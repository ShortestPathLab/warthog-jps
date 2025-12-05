#ifndef JPS_SEARCH_JPS_PRUNE_EXPANSION_POLICY_H
#define JPS_SEARCH_JPS_PRUNE_EXPANSION_POLICY_H

//
// jps_prune_expansion_policy.h
//
// This expansion policy reduces the branching factor
// of a node n during search by ignoring any neighbours which
// could be reached by an equivalent (or shorter) path that visits
// the parent of n but not n itself.
// It extends jps_expansion_policy by pruning the intercardinal nodes
// (NE/NW/SE/SE) by expanding these nodes in-place and pushing their
// cardinal successors.
//
// Used with jump_point_online gives JPS (B+P) algorithm.
// Used with jump_point_offline gives JPS+ (B+P) algorithm.
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

/// @brief generates successor nodes for use in warthog-core search algorithm,
///        pruning intercardinals
/// @tparam JpsJump the jump-point locator to use
/// @tparam InterLimit max distance the intercardinal can expand to, =0 for
///         run-time set, -1 for no limit. Will push nodes first then check if
///         limit is reached, thus jump_point_offline may exceed this greatly.
/// @tparam InterSize the max amount of intercardinal successors.
///         This is where results to the JpsJump are stored, which are placed
///         on the stack.  If this limit is reached, then that intercardinal
///         will be a successor and successors in that direction will cease.
///         The max successors will not exceed to 2*max(W,H).
///
/// JPS expansion policy that pushes all jump points that have a direct
/// line-of-sight to the expanding node following its intercardinal first, then
/// cardinal. It essentially expands the discovered intercardinal successor of
/// JPS without adding those intercardinal nodes to the queue.
///
/// The given JpsJump expects a class from jps::jump namespace,
/// such as jump_point_online for online JPS (B+P), or jump_point_offline<> for
/// offline JPS+ (B+P).
template<typename JpsJump, int16_t InterLimit = -1, size_t InterSize = 1024>
class jps_prune_expansion_policy : public jps_expansion_policy_base
{
	static_assert(InterSize >= 1, "InterSize must be at least 2.");

public:
	/// @brief sets the policy to use with map
	/// @param map point to gridmap, if null map will need to be set later;
	///            otherwise sets map and creates a rotated gridmap.
	///            Use set_map to provide a map at a later stage.
	jps_prune_expansion_policy(warthog::domain::gridmap* map)
	    : jps_expansion_policy_base(map)
	{
		if(map != nullptr)
		{
			jpl_.set_map(rmap_);
			map_width_ = rmap_.map().width();
		}
	}
	using jps_expansion_policy_base::jps_expansion_policy_base;
	~jps_prune_expansion_policy() = default;

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
		    + (sizeof(jps_prune_expansion_policy)
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

	bool
	set_jump_limit(
	    jump::jump_distance limit
	    = std::numeric_limits<jump::jump_distance>::max()) noexcept
	    requires(InterLimit == 0)
	{
		if(limit < 1) return false;
		jump_limit_ = limit;
	}
	jump::jump_distance
	get_jump_limit() const noexcept
	    requires(InterLimit == 0)
	{
		return jump_limit_;
	}
	static constexpr jump::jump_distance
	get_jump_limit() noexcept
	    requires(InterLimit != 0)
	{
		return InterLimit < 0 ? std::numeric_limits<jump::jump_distance>::max()
		                      : static_cast<jump::jump_distance>(InterLimit);
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
	jump::jump_distance jump_limit_
	    = std::numeric_limits<jump::jump_distance>::max();
};

template<typename JpsJump, int16_t InterLimit, size_t InterSize>
void
jps_prune_expansion_policy<JpsJump, InterLimit, InterSize>::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* instance)
{
	reset();

	// compute the direction of travel used to reach the current node.
	const grid_id current_id = grid_id(current->get_id());
	point loc                = rmap_.id_to_point(current_id);
	domain::grid_pair_id pair_id{
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
			    const int32_t node_adj_ic
			        = warthog::grid::dir_id_adj(di, map_width_);
			    const int32_t node_adj_vert
			        = warthog::grid::dir_id_adj_vert(di, map_width_);
			    constexpr int32_t node_adj_hori
			        = warthog::grid::dir_id_adj_hori(di);
			    jump::intercardinal_jump_result res[InterSize];
			    jump::jump_distance inter_total = 0;
			    while(true)
			    {
				    auto [result_n, dist]
				        = jpl_.template jump_intercardinal_many<di>(
				            pair_id, res, InterSize, get_jump_limit());
				    for(decltype(result_n) result_i = 0; result_i < result_n;
				        ++result_i) // jump point
				    {
					    // successful jump
					    const jump::intercardinal_jump_result res_i
					        = res[result_i];
					    assert(res_i.inter > 0);
					    const uint32_t node
					        = current_id.id
					        + static_cast<uint32_t>(
					              node_adj_ic * (inter_total + res_i.inter));
					    const auto cost = warthog::DBL_ROOT_TWO
					        * (inter_total + res_i.inter);
					    assert(rmap_.map().get(
					        pad_id{node})); // successor must be traversable
					    if(res_i.hori > 0)
					    {
						    // horizontal
						    const uint32_t node_j = node
						        + static_cast<uint32_t>(node_adj_hori
						                                * res_i.hori);
						    const auto cost_j
						        = cost + warthog::DBL_ONE * res_i.hori;
						    assert(rmap_.map().get(pad_id{
						        node_j})); // successor must be traversable
						    warthog::search::search_node* jp_succ
						        = this->generate(pad_id{node_j});
						    add_neighbour(jp_succ, cost_j);
					    }
					    if(res_i.vert > 0)
					    {
						    // horizontal
						    const uint32_t node_j = node
						        + static_cast<uint32_t>(node_adj_vert
						                                * res_i.vert);
						    const auto cost_j
						        = cost + warthog::DBL_ONE * res_i.vert;
						    assert(rmap_.map().get(pad_id{
						        node_j})); // successor must be traversable
						    warthog::search::search_node* jp_succ
						        = this->generate(pad_id{node_j});
						    add_neighbour(jp_succ, cost_j);
					    }
				    }
				    if(dist <= 0) // hit wall, break
					    break;
				    if constexpr(InterLimit < 0)
				    {
					    // repeat until all jump points are discovered
					    inter_total += dist;
					    loc          = loc + dist * dir_unit_point(di);
					    pair_id      = rmap_.point_to_pair_id(loc);
				    }
				    else
				    {
					    // reach limit, push dia onto queue
					    const uint32_t node = current_id.id
					        + static_cast<uint32_t>(node_adj_ic * dist);
					    const auto cost = warthog::DBL_ROOT_TWO * dist;
					    assert(rmap_.map().get(
					        pad_id{node})); // successor must be traversable
					    warthog::search::search_node* jp_succ
					        = this->generate(pad_id{node});
					    add_neighbour(jp_succ, cost);
					    break;
				    }
			    }
		    }
	    });
}

template<typename JpsJump, int16_t InterLimit, size_t InterSize>
warthog::search::search_node*
jps_prune_expansion_policy<JpsJump, InterLimit, InterSize>::
    generate_start_node(warthog::search::search_problem_instance* pi)
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

template<typename JpsJump, int16_t InterLimit, size_t InterSize>
warthog::search::search_node*
jps_prune_expansion_policy<JpsJump, InterLimit, InterSize>::
    generate_target_node(warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->target_) >= max_id) { return nullptr; }
	pad_id padded_id = pad_id(pi->target_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

} // namespace jps::search

#endif // JPS_SEARCH_JPS_PRUNE_EXPANSION_POLICY_H
