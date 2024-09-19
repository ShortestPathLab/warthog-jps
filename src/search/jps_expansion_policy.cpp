#include <jps/search/jps_expansion_policy.h>

warthog::jps_expansion_policy::jps_expansion_policy(
    warthog::domain::gridmap* map)
    : expansion_policy(map->height() * map->width())
{
	map_ = map;
	jpl_ = new warthog::online_jump_point_locator(map);
	reset();
}

warthog::jps_expansion_policy::~jps_expansion_policy()
{
	delete jpl_;
}

void
warthog::jps_expansion_policy::expand(
    warthog::search_node* current, warthog::problem_instance* problem)
{
	reset();

	// compute the direction of travel used to reach the current node.
	direction dir_c = warthog::jps::from_direction(
	    (uint32_t)current->get_parent(), (uint32_t)current->get_id(),
	    map_->width());

	// get the tiles around the current node c
	uint32_t c_tiles;
	uint32_t current_id = (uint32_t)current->get_id();
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural
	// and forced neighbour
	uint32_t succ_dirs = warthog::jps::compute_successors(dir_c, c_tiles);
	uint32_t goal_id = (uint32_t)problem->target_;
	// uint32_t search_id = problem->get_searchid();
	for(uint32_t i = 0; i < 8; i++)
	{
		direction d = (direction)(1 << i);
		if(succ_dirs & d)
		{
			warthog::cost_t jumpcost;
			uint32_t succ_id;
			jpl_->jump(d, current_id, goal_id, succ_id, jumpcost);

			if(succ_id != warthog::INF32)
			{
				warthog::search_node* jp_succ = this->generate(succ_id);
				// if(jp_succ->get_searchid() != search_id) {
				// jp_succ->reset(search_id); }
				add_neighbour(jp_succ, jumpcost);
			}
		}
	}
}

uint32_t
warthog::jps_expansion_policy::get_state(warthog::sn_id_t node_id)
{
	return map_->to_unpadded_id(node_id);
}

void
warthog::jps_expansion_policy::print_node(
    warthog::search_node* n, std::ostream& out)
{
	uint32_t x, y;
	map_->to_unpadded_xy(n->get_id(), x, y);
	out << "(" << x << ", " << y << ")...";
	n->print(out);
}

warthog::search_node*
warthog::jps_expansion_policy::generate_start_node(
    warthog::problem_instance* pi)
{
	uint32_t max_id = map_->header_width() * map_->header_height();
	if((uint32_t)pi->start_ >= max_id) { return 0; }
	uint32_t padded_id = map_->to_padded_id((uint32_t)pi->start_);
	if(map_->get_label(padded_id) == 0) { return 0; }
	return generate(padded_id);
}

warthog::search_node*
warthog::jps_expansion_policy::generate_target_node(
    warthog::problem_instance* pi)
{
	uint32_t max_id = map_->header_width() * map_->header_height();
	if((uint32_t)pi->target_ >= max_id) { return 0; }
	uint32_t padded_id = map_->to_padded_id((uint32_t)pi->target_);
	if(map_->get_label(padded_id) == 0) { return 0; }
	return generate(padded_id);
}
