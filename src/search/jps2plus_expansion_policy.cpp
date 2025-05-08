#include <jps/search/jps2plus_expansion_policy.h>

namespace jps::search
{

jps2plus_expansion_policy::jps2plus_expansion_policy(
    warthog::domain::gridmap* map)
    : gridmap_expansion_policy_base(map)
{
	jpl_ = new jump::offline_jump_point_locator2(map);

	costs_.reserve(100);
	jp_ids_.reserve(100);
	reset();
}

jps2plus_expansion_policy::~jps2plus_expansion_policy()
{
	delete jpl_;
}

void
jps2plus_expansion_policy::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* problem)
{
	reset();
	costs_.clear();
	jp_ids_.clear();

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
			jpl_->jump(d, current_id, goal_id, jp_ids_, costs_);
		}
	}

	// uint32_t searchid = problem->get_searchid();
	for(uint32_t i = 0; i < jp_ids_.size(); i++)
	{
		// bits 0-23 store the id of the jump point
		// bits 24-31 store the direction to the parent
		jps_id jp_id = jp_ids_.at(i);
		// TODO: FIX jps node id
		warthog::search::search_node* mynode = generate(jps_id(jp_id.id));
		add_neighbour(mynode, costs_.at(i));
	}
}

warthog::search::search_node*
jps2plus_expansion_policy::generate_start_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	jps_id start    = jps_id(pi->start_);

	if(start.id >= max_id) { return nullptr; }
	jps_id padded_id = jps_id(start);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

warthog::search::search_node*
jps2plus_expansion_policy::generate_target_node(
    warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	jps_id target   = jps_id(pi->target_);

	if(target.id >= max_id) { return nullptr; }
	jps_id padded_id = target;
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

} // namespace jps::search
