#include <jps/search/jpsplus_expansion_policy.h>

namespace jps::search
{

jpsplus_expansion_policy::jpsplus_expansion_policy(
    warthog::domain::gridmap* map)
    : gridmap_expansion_policy_base(map)
{
	jpl_ = new jump::offline_jump_point_locator(map);
}

jpsplus_expansion_policy::~jpsplus_expansion_policy()
{
	delete jpl_;
}

void
jpsplus_expansion_policy::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* problem)
{
	reset();

	// compute the direction of travel used to reach the current node.
	direction dir_c = from_direction(
	    jps_id(current->get_parent()), jps_id(current->get_id()),
	    map_->width());

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
			double jumpcost;
			jps_id succ_id;
			jpl_->jump(d, current_id, goal_id, succ_id, jumpcost);

			if(!succ_id.is_none())
			{
				add_neighbour(this->generate(succ_id), jumpcost);
			}
		}
	}
}

warthog::search::search_node*
jpsplus_expansion_policy::generate_start_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if((uint32_t)pi->start_ >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(pi->start_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

warthog::search::search_node*
jpsplus_expansion_policy::generate_target_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if((uint32_t)pi->target_ >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(pi->target_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

} // namespace jps::search
