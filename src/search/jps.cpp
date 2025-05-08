#include <jps/jump/online_jump_point_locator2.h>
#include <jps/search/jps.h>
#include <warthog/constants.h>
#include <warthog/domain/gridmap.h>

namespace jps::search
{

// computes the forced neighbours of a node.
// for a neighbour to be forced we must check that
// (a) the alt path from the parent is blocked and
// (b) the neighbour is not an obstacle.
// if the test succeeds, we set a bit to indicate
// the direction from the current node (tiles[4])
// to the forced neighbour.
//
// @return an integer value whose lower 8 bits indicate
// the directions of forced neighbours
//
// NB: the first 3 bits of the first 3 bytes of @param tiles represent
// a 3x3 block of nodes. the current node is at the centre of
// the block.
// its NW neighbour is bit 0
// its N neighbour is bit 1
// its NE neighbour is bit 2
// its W neighbour is bit 8
// ...
// its SE neighbour is bit 18
// There are optimisations below that use bitmasks in order
// to speed up forced neighbour computation.
// We use the revised forced neighbour rules described in
// [Harabor and Grastien, The JPS Pathfinding System, SoCS, 2012]
// These rules do not allow diagonal transitions that cut corners.
uint32_t
compute_forced(direction d, uint32_t tiles)
{
	// NB: to avoid branching statements, bit operations are
	// used below to determine which neighbours are traversable
	// and which are obstacles
	enum : uint32_t
	{
		NW = 0b00000000'00000000'00000001,
		N  = 0b00000000'00000000'00000010,
		NE = 0b00000000'00000000'00000100,
		W  = 0b00000000'00000001'00000000,
		X  = 0b00000000'00000010'00000000,
		E  = 0b00000000'00000100'00000000,
		SW = 0b00000001'00000000'00000000,
		S  = 0b00000010'00000000'00000000,
		SE = 0b00000100'00000000'00000000,
	};
	uint32_t ret = 0;
	switch(d)
	{
	case NORTH:
	{
		uint32_t branch_nw = ((tiles & 0b001'00000001'00000000) == 256);
		ret               |= (branch_nw << 3); // force west
		ret               |= (branch_nw << 5); // force northwest

		uint32_t branch_ne = ((tiles & 0b100'00000100'00000000) == 1024);
		ret               |= (branch_ne << 2); // force east
		ret               |= (branch_ne << 4); // force northeast
		break;
	}
	case SOUTH:
	{
		uint32_t branch_sw = ((tiles & 257) == 256);
		ret               |= (branch_sw << 3); // force west
		ret               |= (branch_sw << 7); // force southwest

		uint32_t branch_se = ((tiles & 1028) == 1024);
		ret               |= (branch_se << 2); // force east
		ret               |= (branch_se << 6); // force southeast
		break;
	}
	case EAST:
	{
		uint32_t branch_ne = ((tiles & 3) == 2);
		ret               |= branch_ne;        // force north
		ret               |= (branch_ne << 4); // force northeast

		uint32_t branch_se = ((tiles & 196608) == 131072);
		ret               |= (branch_se << 1); // force south
		ret               |= (branch_se << 6); // force southeast
		break;
	}
	case WEST:
	{
		uint32_t force_nw = ((tiles & 6) == 2);
		ret              |= force_nw;        // force north
		ret              |= (force_nw << 5); // force northwest

		uint32_t force_sw = ((tiles & 393216) == 131072);
		ret              |= (force_sw << 1); // force south
		ret              |= (force_sw << 7); // force southwest
		break;
	}
	default:
		break;
	}
	return ret;
}

// Computes the natural neighbours of a node.
//
// NB: the first 3 bits of the first 3 bytes of @param tiles represent
// a 3x3 block of nodes. the current node is at the centre of
// the block.
// its NW neighbour is bit 0
// its N neighbour is bit 1
// its NE neighbour is bit 2
// its W neighbour is bit 8
// ...
// its SE neighbour is bit 18
// There are optimisations below that use bitmasks in order
// to speed up forced neighbour computation.
uint32_t
compute_natural(direction d, uint32_t tiles)
{
	// In the shift operations below the constant values
	// correspond to bit offsets for direction
	uint32_t ret = 0;
	switch(d)
	{
	case NORTH:
		ret |= (uint32_t)((tiles & 2) == 2) << 0;
		break;
	case SOUTH:
		ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
		break;
	case EAST:
		ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
		break;
	case WEST:
		ret |= (uint32_t)((tiles & 256) == 256) << 3;
		break;
	case NORTHWEST:
		ret |= (uint32_t)((tiles & 2) == 2) << 0;
		ret |= (uint32_t)((tiles & 256) == 256) << 3;
		ret |= (uint32_t)((tiles & 259) == 259) << 5;
		break;
	case NORTHEAST:
		ret |= (uint32_t)((tiles & 2) == 2) << 0;
		ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
		ret |= (uint32_t)((tiles & 1030) == 1030) << 4;
		break;
	case SOUTHWEST:
		ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
		ret |= (uint32_t)((tiles & 256) == 256) << 3;
		ret |= (uint32_t)((tiles & 196864) == 196864) << 7;
		break;
	case SOUTHEAST:
		ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
		ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
		ret |= (uint32_t)((tiles & 394240) == 394240) << 6;
		break;
	default:
		ret |= (uint32_t)((tiles & 2) == 2) << 0;
		ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
		ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
		ret |= (uint32_t)((tiles & 256) == 256) << 3;
		ret |= (uint32_t)((tiles & 259) == 259) << 5;
		ret |= (uint32_t)((tiles & 1030) == 1030) << 4;
		ret |= (uint32_t)((tiles & 196864) == 196864) << 7;
		ret |= (uint32_t)((tiles & 394240) == 394240) << 6;
		break;
	}
	return ret;
}

#if 0
warthog::graph::xy_graph*
create_jump_point_graph(warthog::domain::gridmap* gm)
{
	warthog::graph::xy_graph* graph = new warthog::graph::xy_graph();
	warthog::jps::online_jump_point_locator2 jpl(gm);
	uint32_t mapwidth = gm->header_width();
	uint32_t mapheight = gm->header_height();
	std::unordered_map<uint32_t, uint32_t> id_map;

	// add nodes to graph
	for(uint32_t y = 0; y < mapheight; y++)
	{
		for(uint32_t x = 0; x < mapwidth; x++)
		{
			uint32_t from_id = gm->to_padded_id(y * mapwidth + x);
			if(!gm->get_label(gm->to_padded_id(x, y))) { continue; }

			uint32_t w_id = from_id - 1;
			uint32_t e_id = from_id + 1;
			uint32_t s_id = from_id + gm->width();
			uint32_t n_id = from_id - gm->width();
			uint32_t nw_id = (from_id - gm->width()) - 1;
			uint32_t ne_id = (from_id - gm->width()) + 1;
			uint32_t sw_id = (from_id + gm->width()) - 1;
			uint32_t se_id = (from_id + gm->width()) + 1;

			// detect all corner turning points (== jump points)
			// and add them to the jump point graph
			uint32_t tiles;
			gm->get_neighbours(from_id, (uint8_t*)&tiles);
			if((!gm->get_label(nw_id) && gm->get_label(w_id)
			    && gm->get_label(n_id))
			   || (!gm->get_label(ne_id) && gm->get_label(e_id)
			       && gm->get_label(n_id))
			   || (!gm->get_label(se_id) && gm->get_label(e_id)
			       && gm->get_label(s_id))
			   || (!gm->get_label(sw_id) && gm->get_label(w_id)
			       && gm->get_label(s_id)))
			{
				uint32_t graph_id = graph->add_node((int32_t)x, (int32_t)y);
				id_map.insert(
				    std::pair<uint32_t, uint32_t>(from_id, graph_id));
			}
		}
	}

	// add edges to graph
	for(uint32_t from_id = 0; from_id < graph->get_num_nodes(); from_id++)
	{
		int32_t x, y;
		graph->get_xy(from_id, x, y);
		uint32_t gm_id
		    = gm->to_padded_id((uint32_t)y * mapwidth + (uint32_t)x);
		warthog::graph::node* from = graph->get_node(from_id);

		for(uint32_t i = 0; i < 8; i++)
		{
			direction d = (direction)(1 << i);
			std::vector<uint32_t> jpoints;
			std::vector<double> jcosts;
			jpl.jump(d, gm_id, warthog::INF32, jpoints, jcosts);
			for(uint32_t idx = 0; idx < jpoints.size(); idx++)
			{
				uint32_t jp_id = jpoints[idx] & ((1 << 24) - 1);
				// direction d =
				// (direction)(jpoints[idx] >> 24);
				std::unordered_map<uint32_t, uint32_t>::iterator it_to_id;
				it_to_id = id_map.find(jp_id);
				assert(it_to_id != id_map.end());
				uint32_t to_id = it_to_id->second;
				warthog::graph::node* to = graph->get_node(to_id);
				from->add_outgoing(
				    warthog::graph::edge(to_id, (uint32_t)jcosts[idx]));
				to->add_outgoing(
				    warthog::graph::edge(from_id, (uint32_t)jcosts[idx]));
			}
		}
	}
	return graph;
}
#endif

warthog::domain::gridmap*
create_corner_map(warthog::domain::gridmap* gm)
{
	uint32_t mapwidth  = gm->header_width();
	uint32_t mapheight = gm->header_height();
	warthog::domain::gridmap* corner_map
	    = new warthog::domain::gridmap(mapheight, mapwidth);

	uint32_t gmwidth = gm->width();

	// add nodes to graph
	for(uint32_t y = 0; y < mapheight; y++)
	{
		for(uint32_t x = 0; x < mapwidth; x++)
		{
			jps_id from_id = jps_id(gm->to_padded_id_from_unpadded(x, y));
			if(!gm->get_label(from_id)) { continue; }

			jps_id w_id  = jps_id(from_id.id - 1);
			jps_id e_id  = jps_id(from_id.id + 1);
			jps_id s_id  = jps_id(from_id.id + gmwidth);
			jps_id n_id  = jps_id(from_id.id - gmwidth);
			jps_id nw_id = jps_id((from_id.id - gmwidth) - 1);
			jps_id ne_id = jps_id((from_id.id - gmwidth) + 1);
			jps_id sw_id = jps_id((from_id.id + gmwidth) - 1);
			jps_id se_id = jps_id((from_id.id + gmwidth) + 1);

			// detect all corner turning points (== jump points)
			// and add them to the jump point graph
			uint32_t tiles;
			gm->get_neighbours(from_id, (uint8_t*)&tiles);
			if((!gm->get_label(nw_id) && gm->get_label(w_id)
			    && gm->get_label(n_id))
			   || (!gm->get_label(ne_id) && gm->get_label(e_id)
			       && gm->get_label(n_id))
			   || (!gm->get_label(se_id) && gm->get_label(e_id)
			       && gm->get_label(s_id))
			   || (!gm->get_label(sw_id) && gm->get_label(w_id)
			       && gm->get_label(s_id)))
			{
				corner_map->set_label(from_id, true);
			}
		}
	}
	return corner_map;
}

} // namespace jps::search
