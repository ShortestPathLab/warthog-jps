#ifndef JPS_SEARCH_JPS_PRUNE_EXPANSION_POLICY_H
#define JPS_SEARCH_JPS_PRUNE_EXPANSION_POLICY_H

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
/// @tparam InterLimit max length the intercardinal can expand to, =0 for run-time set, -1 to prune all intercardinal points
/// @tparam InterSize size of intercardinal successor array, is stored on the stack.
///                   If this successor count is reached withing InterLimit, then end successor unless InterLimit<0
///
/// JPS expansion policy that pushes the first cardinal and intercardinal
/// jump point, block-based jumping is the standard jump used by jump_point_online.
/// jps_2011_expansion_policy<jump_point_online> gives JPS (B).
/// jps_2011_expansion_policy<jump_point_offline> gives JPS+.
template<typename JpsJump, int16_t InterLimit = -1, size_t InterSize = 1024>
class jps_prune_expansion_policy
    : public warthog::search::gridmap_expansion_policy_base
{
	static_assert(InterSize >= 1, "InterSize must be at least 2.");
public:
	jps_expansion_policy(warthog::domain::gridmap* map)
	    : gridmap_expansion_policy_base(map)
	{
		if (map != nullptr) {
			set_map(*map);
		}
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

	void set_map(warthog::domain::gridmap& map)
	{
		rmap_.create_rmap(map);
		jpl_.set_map(rmap_);
		map_width_ = rmap_.map().width();
		// std::ofstream map1("map1.txt");
		// std::ofstream map2("map2.txt");
		// rmap_.map().print(map1);
		// rmap_.rmap().print(map2);
	}
	void set_map(domain::gridmap_rotate_ptr rmap)
	{
		rmap_.link(rmap);
		jpl_.set_map(rmap_);
		map_width_ = rmap_.map().width();
	}

private:
	domain::rotate_gridmap rmap_;
	JpsJump jpl_;
	point target_loc_ = {};
	grid_id target_id_ = {};
	uint32_t map_width_ = 0;
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
	const point loc = rmap_.id_to_point(current_id);
	assert(rmap_.map().get_label(current_id) && rmap_.map().get_label(rmap_.point_to_id_d<EAST_ID>(loc))); // loc must be trav on map
	assert(rmap_.rmap().get_label(pad_id(rmap_.point_to_id_d<NORTH_ID>(loc).id))); // loc must be trav on rmap
	// const jps_rid current_rid = jpl_.id_to_rid(current_id);
	// const cost_t current_cost = current->get_g();
	const direction dir_c = from_direction(
	    grid_id(current->get_parent()), current_id, rmap_.map().width());
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
	>([&](auto iv) {
		constexpr direction_id di = decltype(iv)::value;
		if(succ_dirs & warthog::grid::to_dir(di))
		{
			auto jump_result = jpl_.template jump_cardinal_next<di>(loc);
			if(jump_result > 0) // jump point
			{
				// successful jump
				pad_id node{static_cast<uint32_t>(current_id.id + warthog::grid::dir_id_adj(di, map_width_) * jump_result)};
				assert(rmap_.map().get(node)); // successor must be traversable
				warthog::search::search_node* jp_succ = this->generate(node);
				add_neighbour(jp_succ, jump_result * warthog::DBL_ONE);
			}
		}
	});
	// intercardinal directions
	::warthog::util::for_each_integer_sequence<
		std::integer_sequence<direction_id, NORTHEAST_ID, NORTHWEST_ID, SOUTHEAST_ID, SOUTHWEST_ID>
	>([&](auto iv) {
		constexpr direction_id di = decltype(iv)::value;
		if(succ_dirs & warthog::grid::to_dir(di))
		{
			jump::intercardinal_jump_result res;
			auto jump_result = jpl_.template jump_intercardinal_many<di>(loc, &res, 1);
			if(jump_result.first > 0) // jump point
			{
				// successful jump
				pad_id node{pad_id(static_cast<int32_t>(current_id.id + warthog::grid::dir_id_adj(di, map_width_) * res.inter))};
				assert(rmap_.map().get(node)); // successor must be traversable
				warthog::search::search_node* jp_succ = this->generate(node);
				add_neighbour(jp_succ, res.inter * warthog::DBL_ROOT_TWO);
			}
		}
	});
}

template<typename JpsJump, int16_t InterLimit, size_t InterSize>
warthog::search::search_node*
jps_prune_expansion_policy<JpsJump, InterLimit, InterSize>::generate_start_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->start_) >= max_id) { return nullptr; }
	pad_id padded_id = pad_id(pi->start_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	target_id_ = grid_id(pi->target_);
	uint32_t x, y;
	rmap_.map().to_unpadded_xy(target_id_, x, y);
	target_loc_.x = static_cast<uint16_t>(x);
	target_loc_.y = static_cast<uint16_t>(y);
	return generate(padded_id);
}

template<typename JpsJump, int16_t InterLimit, size_t InterSize>
warthog::search::search_node*
jps_prune_expansion_policy<JpsJump, InterLimit, InterSize>::generate_target_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->target_) >= max_id) { return nullptr; }
	pad_id padded_id = pad_id(pi->target_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

}

#endif // JPS_SEARCH_JPS_PRUNE_EXPANSION_POLICY_H
