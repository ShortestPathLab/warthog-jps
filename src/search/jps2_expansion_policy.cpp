#include <jps/search/jps2_expansion_policy.h>

namespace jps::search
{

jps2_expansion_policy::jps2_expansion_policy(warthog::domain::gridmap* map)
    : gridmap_expansion_policy_base(map)
{
	jpl_ = new jump::online_jump_point_locator2(map);
	jp_ids_.reserve(100);
}

jps2_expansion_policy::~jps2_expansion_policy()
{
	delete jpl_;
}

void
jps2_expansion_policy::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* problem)
{
	reset();
	jp_ids_.clear();
	jp_costs_.clear();

	// compute the direction of travel used to reach the current node.
	// TODO: store this value with the jump point location so we don't need
	// to compute it all the time
	jps_id p_id     = jps_id(current->get_parent());
	jps_id c_id     = jps_id(current->get_id());
	direction dir_c = from_direction(p_id, c_id, map_->width());

	// get the tiles around the current node c
	uint32_t c_tiles;
	jps_id current_id = jps_id(current->get_id());
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural
	// and forced neighbour
	uint32_t succ_dirs = compute_successors(dir_c, c_tiles);
	jps_id goal_id     = jps_id(problem->target_);

	for(uint32_t i = 0; i < 8; i++)
	{
		direction d = (direction)(1 << i);
		if(succ_dirs & d)
		{
			jpl_->jump(d, current_id, goal_id, jp_ids_, jp_costs_);
		}
	}

	// uint32_t searchid = problem->get_searchid();
	for(uint32_t i = 0; i < jp_ids_.size(); i++)
	{
		// bits 0-23 store the id of the jump point
		// bits 24-31 store the direction to the parent
		jps_id jp_id            = jp_ids_.at(i);
		warthog::cost_t jp_cost = jp_costs_.at(i);

		warthog::search::search_node* mynode = generate(jp_id);
		add_neighbour(mynode, jp_cost);
	}
}

// void
// jps2_expansion_policy::update_parent_direction(warthog::search::search_node*
// n)
//{
//     uint32_t jp_id = jp_ids_.at(this->get_current_successor_index());
//     assert(n->get_id() == (jp_id & warthog::jps::JPS_ID_MASK));
//     direction pdir =
//         (direction)*(((uint8_t*)(&jp_id))+3);
//     n->set_pdir(pdir);
// }

warthog::search::search_node*
jps2_expansion_policy::generate_start_node(
    warthog::search::search_problem_instance* pi)
{
	jps_id start    = jps_id(pi->start_);
	uint32_t max_id = map_->width() * map_->height();

	if(start.id >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(start);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

warthog::search::search_node*
jps2_expansion_policy::generate_target_node(
    warthog::search::search_problem_instance* pi)
{
	jps_id target   = jps_id(pi->target_);
	uint32_t max_id = map_->width() * map_->height();

	if(target.id >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(target);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

} // namespace jps::search
