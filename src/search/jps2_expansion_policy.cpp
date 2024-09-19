#include <jps/search/jps2_expansion_policy.h>

warthog::jps2_expansion_policy::jps2_expansion_policy(
    warthog::domain::gridmap* map)
    : expansion_policy(map->height() * map->width())
{
	map_ = map;
	jpl_ = new warthog::jps::online_jump_point_locator2(map);
	jp_ids_.reserve(100);
}

warthog::jps2_expansion_policy::~jps2_expansion_policy()
{
	delete jpl_;
}

void
warthog::jps2_expansion_policy::expand(
    warthog::search_node* current, warthog::problem_instance* problem)
{
	reset();
	jp_ids_.clear();
	jp_costs_.clear();

	// compute the direction of travel used to reach the current node.
	// TODO: store this value with the jump point location so we don't need
	// to compute it all the time
	uint32_t p_id = current->get_parent();
	uint32_t c_id = current->get_id();
	direction dir_c = warthog::jps::from_direction(p_id, c_id, map_->width());

	// get the tiles around the current node c
	uint32_t c_tiles;
	uint32_t current_id = (uint32_t)current->get_id();
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural
	// and forced neighbour
	uint32_t succ_dirs = warthog::jps::compute_successors(dir_c, c_tiles);
	uint32_t goal_id = (uint32_t)problem->target_;

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
		uint32_t jp_id = jp_ids_.at(i);
		warthog::cost_t jp_cost = jp_costs_.at(i);

		warthog::search_node* mynode = generate(jp_id);
		add_neighbour(mynode, jp_cost);
	}
}

// void
// warthog::jps2_expansion_policy::update_parent_direction(warthog::search_node*
// n)
//{
//     uint32_t jp_id = jp_ids_.at(this->get_current_successor_index());
//     assert(n->get_id() == (jp_id & warthog::jps::JPS_ID_MASK));
//     direction pdir =
//         (direction)*(((uint8_t*)(&jp_id))+3);
//     n->set_pdir(pdir);
// }

uint32_t
warthog::jps2_expansion_policy::get_state(warthog::sn_id_t node_id)
{
	return map_->to_unpadded_id(node_id);
}

void
warthog::jps2_expansion_policy::print_node(
    warthog::search_node* n, std::ostream& out)
{
	uint32_t x, y;
	map_->to_unpadded_xy(n->get_id(), x, y);
	out << "(" << x << ", " << y << ")...";
	n->print(out);
}

warthog::search_node*
warthog::jps2_expansion_policy::generate_start_node(
    warthog::problem_instance* pi)
{
	uint32_t start = (uint32_t)pi->start_;
	uint32_t max_id = map_->header_width() * map_->header_height();

	if(start >= max_id) { return 0; }
	uint32_t padded_id = map_->to_padded_id(start);
	if(map_->get_label(padded_id) == 0) { return 0; }
	return generate(padded_id);
}

warthog::search_node*
warthog::jps2_expansion_policy::generate_target_node(
    warthog::problem_instance* pi)
{
	uint32_t target = (uint32_t)pi->target_;
	uint32_t max_id = map_->header_width() * map_->header_height();

	if(target >= max_id) { return 0; }
	uint32_t padded_id = map_->to_padded_id(target);
	if(map_->get_label(padded_id) == 0) { return 0; }
	return generate(padded_id);
}
