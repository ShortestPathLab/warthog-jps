#include <jps/search/jps_expansion_policy.h>

namespace jps::search
{

jps_expansion_policy::jps_expansion_policy(warthog::domain::gridmap* map)
    : gridmap_expansion_policy_base(map)
{
	jpl_ = new jump::online_jump_point_locator(map);
	reset();
}

jps_expansion_policy::~jps_expansion_policy()
{
	delete jpl_;
}

void
jps_expansion_policy::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* problem)
{
	reset();

	// compute the direction of travel used to reach the current node.
	jps_id current_id = jps_id(current->get_id());
	direction dir_c   = from_direction(
        jps_id(current->get_parent()), current_id, map_->width());

	// get the tiles around the current node c
	uint32_t c_tiles;
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural
	// and forced neighbour
	uint32_t succ_dirs = compute_successors(dir_c, c_tiles);
	jps_id goal_id     = jps_id(problem->target_);
	// uint32_t search_id = problem->get_searchid();
	for(uint32_t i = 0; i < 8; i++)
	{
		direction d = (direction)(1 << i);
		if(succ_dirs & d)
		{
			warthog::cost_t jumpcost;
			jps_id succ_id;
			jpl_->jump(d, current_id, goal_id, succ_id, jumpcost);

			if(!succ_id.is_none())
			{
				warthog::search::search_node* jp_succ
				    = this->generate(succ_id);
				// if(jp_succ->get_searchid() != search_id) {
				// jp_succ->reset(search_id); }
				add_neighbour(jp_succ, jumpcost);
			}
		}
	}
}

warthog::search::search_node*
jps_expansion_policy::generate_start_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->start_) >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(pi->start_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

warthog::search::search_node*
jps_expansion_policy::generate_target_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->target_) >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(pi->target_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

} // namespace jps::search
