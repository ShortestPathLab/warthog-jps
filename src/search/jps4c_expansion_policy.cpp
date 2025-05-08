#include <jps/search/jps4c_expansion_policy.h>

namespace jps::search
{

jps4c_expansion_policy::jps4c_expansion_policy(warthog::domain::gridmap* map)
    : gridmap_expansion_policy_base(map)
{
	jpl_ = new jump::four_connected_jps_locator(map);
	reset();
}

jps4c_expansion_policy::~jps4c_expansion_policy()
{
	delete jpl_;
}

void
jps4c_expansion_policy::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* problem)
{
	reset();

	jps_id current_id = jps_id(current->get_id());
	jps_id parent_id  = jps_id(current->get_parent());
	jps_id goal_id    = jps_id(problem->target_);

	// compute the direction of travel used to reach the current node.
	direction dir_c = from_direction_4c(parent_id, current_id, map_->width());
	assert(
	    dir_c == NONE || dir_c == NORTH || dir_c == SOUTH || dir_c == EAST
	    || dir_c == WEST);

	// get the tiles around the current node c and determine
	// which of the available moves are forced and which are natural
	uint32_t c_tiles;
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);
	uint32_t succ_dirs = compute_successors_4c(dir_c, c_tiles);

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
jps4c_expansion_policy::generate_start_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(pi->start_.id >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(pi->start_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

warthog::search::search_node*
jps4c_expansion_policy::generate_target_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(pi->target_.id >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(pi->target_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

} // namespace jps::search
