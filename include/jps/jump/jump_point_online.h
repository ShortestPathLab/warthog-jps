#ifndef JPS_JUMP_JUMP_POINT_ONLINE_H
#define JPS_JUMP_JUMP_POINT_ONLINE_H

// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011,
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//

#include <jps/forward.h>
#include <warthog/domain/gridmap.h>
#include <array>
#include <memory>

namespace jps::jump
{

struct intercardinal_jump_result
{
	jps_id node;
	jps_rid rnode;
	uint32_t dist;
};

namespace details
{

/// @brief 
/// @tparam East Jump east (true) or west (false)
/// @param map 
/// @param node 
/// @param goal 
/// @return 
template <bool East>
uint32_t jump_point_online_hori(const ::warthog::domain::gridmap& map, uint32_t node, uint32_t goal)
{
	// read tiles from the grid:
	// - along the row of node_id
	// - from the row above node_id
	// - from the row below node_id
	// NB: the jump direction corresponds to moving from the
	// low bit of the tileset and towards the high bit (EAST)
	// or high bit to low bit (WEST)
	auto nei_slider = map.get_neighbours_slider(pad_id{node});
	nei_slider.adj_bytes(East ? -1 : -6); // current location is 1 byte from boundrary
	// width8_bits is how many bits in on word current node is at, from lsb EAST and from msb WEST
	nei_slider.width8_bits = East ? nei_slider.width8_bits + 8 : 15 - nei_slider.width8_bits;
		// 15 - width8_bits == 63 - (width8_bits + 6 * 8)
	uint32_t jump_count = 0;

	// order going east is stored as least significant bit to most significant bit

	while(true)
	{
		std::array<uint64_t, 3> neis = nei_slider.get_neighbours_64bit_le();
		assert(nei_slider.width8_bits < 16);
		uint64_t tmp = East ? ~(~0ull << nei_slider.width8_bits)
			: ~(~0ull >> nei_slider.width8_bits);
		// shift above and below 2 points east
		// mask out to trav(1) before node
		neis[0] |= tmp;
		neis[1] |= tmp;
		neis[2] |= tmp;
		//   v  & will make this the least sig bit, and is where the jump point is located
		// 10100011 : 0
		// 10111000 : <<1~
		if constexpr (East) {
			tmp = ((~neis[1] << 1) & neis[1]) // above row: block(zero) trailing trav(one)
				| ((~neis[2] << 1) & neis[2]); // below row: block(zero) trailing trav(one)
		} else {
			tmp = ((~neis[1] >> 1) & neis[1]) // above row: block(zero) trailing trav(one)
				| ((~neis[2] >> 1) & neis[2]); // below row: block(zero) trailing trav(one)
		}
		tmp = tmp | ~neis[0];
		if (tmp) {
			int stop_pos = East ? std::countr_zero(tmp) : std::countl_zero(tmp); // the location to stop at
			//  v is blocker location, prune unless target is present
			// 10111111
			// dead end takes president as jump point can't pass a blocker
			jump_count += static_cast<uint32_t>(stop_pos) - nei_slider.width8_bits;
			uint32_t goal_jump = East ? goal - node : node - goal;
			// if blocked: pos + jump_count = first block
			//  otherwise: jump_count = trav cell after turn (jump point location)
			// check for goal with goal_jump (dist) <= jump_count, as if < than goal is reachable,
			// if equal then trav pos is the goal
			// if greater, than goal is further or another row or behind (as unsigned)
			if (goal_jump <= jump_count) {
				// goal reached
				jump_count = goal_jump;
			} else if ( East ?
				!(neis[0] & (1u << stop_pos)) :
				!(neis[0] & (static_cast<uint64_t>(std::numeric_limits<int64_t>::min()) >> stop_pos))
				) // deadend
			{
				jump_count = 0;
			}
			return jump_count;
		}

		// failed, goto next 56 bits
		jump_count += 63 - nei_slider.width8_bits;
		nei_slider.adj_bytes(East ? 7 : -7);
		nei_slider.width8_bits = 7;
	}
}

}

template <JpsFeature Feature = JpsFeature::DEFAULT>
class jump_point_online
{
public:
	using gridmap = warthog::domain::gridmap;
	jump_point_online();
	jump_point_online(gridmap* map)
	{
		if (map != nullptr)
			set_map(*map);
	}
	~jump_point_online() = default;

	static consteval bool feature_prune_intercardinal() noexcept { return (static_cast<uint32_t>(Feature) & static_cast<uint32_t>(JpsFeature::PRUNE_INTERCARDINAL)) != 0; }
	static consteval bool feature_store_cardinal() noexcept { return (static_cast<uint32_t>(Feature) & static_cast<uint32_t>(JpsFeature::STORE_CARDIANL_JUMP)) != 0; }

	void set_map(gridmap& map);
	void set_goal(jps_id goal_id) noexcept;
	void set_goal(point goal_id) noexcept;

	const gridmap* get_map() const noexcept
	{
		return map_;
	}
	gridmap* get_map() noexcept
	{
		return map_;
	}
	const gridmap* get_rmap() const noexcept
	{
		return rmap_.get();
	}
	gridmap* get_rmap() noexcept
	{
		return rmap_.get();
	}

	/**
	 * @returns pair first: steps to reach node (or 0 if no node), second: id of jump point (or node_id if none)
	 */
	std::pair<uint32_t, jps_id>
	jump_cardinal(direction d, jps_id node_id, jps_rid rnode_id);
	intercardinal_jump_result
	jump_intercardinal(direction d, jps_id node_id, jps_rid rnode_id, jps_id* result_node, cost_t* result_cost, uint32_t result_size = 0);
	
	uint32_t jump_north(jps_rid rnode)
	{
		return jump_east(*rmap_, rnode.id, rgoal_.id);
	}
	uint32_t jump_east(jps_id node)
	{
		return jump_east(*map_, node.id, goal_.id);
	}
	uint32_t jump_south(jps_rid rnode)
	{
		return jump_west(*rmap_, rnode.id, rgoal_.id);
	}
	uint32_t jump_west(jps_id node)
	{
		return jump_west(*map_, node.id, goal_.id);
	}
	intercardinal_jump_result jump_northeast(jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost, uint32_t result_size = 0)
	{
		return jump_intercardinal<NORTHEAST>(node, rnode, result_node, result_cost, result_size);
	}
	intercardinal_jump_result jump_southeast(jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost, uint32_t result_size)
	{
		return jump_intercardinal<SOUTHEAST>(node, rnode, result_node, result_cost, result_size);
	}
	intercardinal_jump_result jump_southwest(jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost, uint32_t result_size)
	{
		return jump_intercardinal<SOUTHWEST>(node, rnode, result_node, result_cost, result_size);
	}
	intercardinal_jump_result jump_northwest(jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost, uint32_t result_size)
	{
		return jump_intercardinal<NORTHWEST>(node, rnode, result_node, result_cost, result_size);
	}

	size_t
	mem()
	{
		return sizeof(this) + rmap_->mem();
	}

	point point_to_rpoint(point p) const noexcept
	{
		return {static_cast<uint16_t>(map_unpad_height_m1_ - p.y), static_cast<uint16_t>(p.x)};
	}
	point rpoint_to_point(point p) const noexcept
	{
		return {static_cast<uint16_t>(p.y), static_cast<uint16_t>(map_unpad_height_m1_ - p.x)};
	}
	jps_id point_to_id(point p) const noexcept
	{
		return jps_id{static_cast<jps_id::id_type>(p.y + gridmap::PADDED_ROWS) * map_width_ + static_cast<jps_id::id_type>(p.x)};
	}
	jps_rid rpoint_to_rid(point p) const noexcept
	{
		return jps_rid{static_cast<jps_rid::id_type>(p.y + gridmap::PADDED_ROWS) * rmap_width_ + static_cast<jps_rid::id_type>(p.x)};
	}
	point id_to_point(jps_id p) const noexcept
	{
		return {static_cast<uint16_t>(p.id % map_width_), static_cast<uint16_t>(p.id / map_width_ - gridmap::PADDED_ROWS)};
	}
	point rid_to_rpoint(jps_rid p) const noexcept
	{
		return {static_cast<uint16_t>(p.id % rmap_width_), static_cast<uint16_t>(p.id / rmap_width_ - gridmap::PADDED_ROWS)};
	}
	jps_rid
	id_to_rid(jps_id mapid)
	{
		assert(!mapid.is_none());
		return rpoint_to_rid(point_to_rpoint(id_to_point(mapid)));
	}
	jps_id
	rid_to_id(jps_rid mapid)
	{
		assert(!mapid.is_none());
		return point_to_id(rpoint_to_point(rid_to_rpoint(mapid)));
	}

protected:

	static uint32_t jump_east(const gridmap& map, uint32_t node, uint32_t goal);
	static uint32_t jump_west(const gridmap& map, uint32_t node, uint32_t goal);

	/**
	 * Jumps on the intercardinal.
	 * The result_* variables store the cardinal jump results (if enabled)
	 * result_count only used for PRUNE_INTERCARDINAL.
	 * result_* must be big enough to store:
	 * !PRUNE_INTERCARDINAL & !STORE_CARDIANL_JUMP => 0 (should be nullptr)
	 * !PRUNE_INTERCARDINAL & STORE_CARDIANL_JUMP => 2
	 * PRUNE_INTERCARDINAL => result_cost (min 4)
	 * 
	 * if !PRUNE_INTERCARDINAL & STORE_CARDIANL_JUMP:
	 *   results[0] = east/west result or jps_id::none() if none
	 *   results[1] = north/south result or jps_id::none() if none
	 * 
	 * The return intercardinal_jump_result is as follows:
	 * node: return end point id, or jps_id::none() if no more successors.
	 * rnode: return end rpoint id, or jps_rid::none() if no more successors.
	 * dist: !PRUNE_INTERCARDINAL => distance jumped (0 = no jump)
	 *        PRUNE_INTERCARDINAL => the number of elements pushed on the result node
	 */
	template <direction D>
	intercardinal_jump_result jump_intercardinal(jps_id node, jps_rid rnode, jps_id* result_node, cost_t* result_cost, uint32_t result_size = 0);

	// jps_id point_to_jps_id(point p) noexcept
	// {
	// 	return 
	// }

protected:
	/**
	 * Rotate 90 clockwise
	 * NORTH -> EAST
	 * EAST -> SOUTH
	 * SOUTH -> WEST
	 * WEST -> NORTH
	 * NORTHEAST -> SOUTHEAST
	 * SOUTHEAST -> SOUTHWEST
	 * SOUTHWEST -> NORTHWEST
	 * NORTHWEST -> NORTHEAST
	 * 
	 * unpadded (x,y) -> (y, Rh-1-x)
	 */
	void
	create_rotate_(const gridmap& orig);

protected:
	gridmap* map_ = {};
	std::unique_ptr<gridmap> rmap_;
	uint32_t map_width_ = 0;
	uint32_t rmap_width_ = 0;
	uint32_t map_unpad_height_m1_ = 0;
	jps_id goal_ = {};
	jps_rid rgoal_ = {};
};

template <JpsFeature Feature>
void jump_point_online<Feature>::set_map(gridmap& orig)
{
	map_ = &orig;
	create_rotate_(orig);
}

template <JpsFeature Feature>
void jump_point_online<Feature>::set_goal(jps_id p) noexcept
{
	goal_ = p;
	rgoal_ = id_to_rid(p);
}
template <JpsFeature Feature>
void jump_point_online<Feature>::set_goal(point p) noexcept
{
	goal_ = point_to_id(p);
	rgoal_ = rpoint_to_rid(point_to_rpoint(p));
}

template <JpsFeature Feature>
void
jump_point_online<Feature>::create_rotate_(const gridmap& orig)
{
	const uint32_t maph = map_->header_height();
	const uint32_t mapw = map_->header_width();
	auto tmap = std::make_unique<gridmap>(mapw, maph);

	for(uint32_t x = 0; x < mapw; x++)
	{
		for(uint32_t y = 0; y < maph; y++)
		{
			bool label
				= map_->get_label(map_->to_padded_id_from_unpadded(x, y));
			uint32_t rx = (maph-1) - y;
			uint32_t ry = x;
			tmap->set_label(tmap->to_padded_id_from_unpadded(rx, ry), label);
		}
	}
	
	// set values
	rmap_ = std::move(tmap);
	map_width_ = map_->width();
	rmap_width_ = rmap_->width();
	map_unpad_height_m1_ = maph-1;
}


template <JpsFeature Feature>
std::pair<uint32_t, jps_id>
jump_point_online<Feature>::jump_cardinal(direction d, jps_id node_id, jps_rid rnode_id)
{
	std::pair<uint32_t, jps_id> node;
	switch (d) {
	case NORTH:
		node.first = jump_north(rnode_id);
		node.second.id = node_id.id - map_width_ * node.first;
		break;
	case SOUTH:
		node.first = jump_south(rnode_id);
		node.second.id = node_id.id + map_width_ * node.first;
		break;
	case EAST:
		node.first = jump_east(node_id);
		node.second.id = node_id.id + node.first;
		break;
	case WEST:
		node.first = jump_west(node_id);
		node.second.id = node_id.id - node.first;
		break;
	default:
		assert(false);
		node = {0,node_id};
	}
	return node;
}
template <JpsFeature Feature>
intercardinal_jump_result
jump_point_online<Feature>::jump_intercardinal(direction d, jps_id node_id, jps_rid rnode_id, jps_id* result_node, cost_t* result_cost, uint32_t result_size)
{
	intercardinal_jump_result node;
	switch (d) {
	case NORTHEAST:
		node = jump_intercardinal<NORTHEAST>(node_id, rnode_id, result_node, result_cost, result_size);
		break;
	case NORTHWEST:
		node = jump_intercardinal<NORTHWEST>(node_id, rnode_id, result_node, result_cost, result_size);
		break;
	case SOUTHEAST:
		node = jump_intercardinal<SOUTHEAST>(node_id, rnode_id, result_node, result_cost, result_size);
		break;
	case SOUTHWEST:
		node = jump_intercardinal<SOUTHWEST>(node_id, rnode_id, result_node, result_cost, result_size);
		break;
	default:
		assert(false);
		node = {jps_id::none(), jps_rid::none(), 0};
	}
	return node;
}

template <JpsFeature Feature>
uint32_t jump_point_online<Feature>::jump_east(const gridmap& map, uint32_t node, uint32_t goal)
{
	return details::jump_point_online_hori<true>(map, node, goal);
}

template <JpsFeature Feature>
uint32_t jump_point_online<Feature>::jump_west(const gridmap& map, uint32_t node, uint32_t goal)
{
	return details::jump_point_online_hori<false>(map, node, goal);
}

template <JpsFeature Feature>
template <direction D>
intercardinal_jump_result jump_point_online<Feature>::jump_intercardinal(jps_id node, jps_rid rnode, jps_id* result_node[[maybe_unused]], cost_t* result_cost[[maybe_unused]], uint32_t result_size[[maybe_unused]])
{
	static_assert(D == NORTHEAST || D == NORTHWEST || D == SOUTHEAST || D == SOUTHWEST, "D must be inter-cardinal.");
	/*
	map:
	NW N NE
	 W   E
	SW S SE

	rmap = rotate 90 CW:
	SW W NW
	 S   N
	SE E NE

	SE = jump_intercardinal_pos(M0=map,M1=rmap)
	map:  x+=1, y+=1, pos += mapW+1
	rmap: x-=1, y+=1, pos += rmapW-1
	jump_south = rmap: jump_west(M1), (x,y+r)
	jump_east = map: jump_east(M0), (x+r,y)

	NE = jump_intercardinal_pos(M0=rmap,M1=map)
	map:  x+=1, y-=1, pos -= mapW-1
	rmap: x+=1, y+=1, pos += rmapW+1
	jump_east = map: jump_east(M1), (x+r,y)
	jump_north = rmap: jump_east(M0), (x,y-r)

	NW = jump_intercardinal_neg(M0=map,M1=rmap)
	map:  x-=1, y-=1, pos -= mapW+1
	rmap: x+=1, y-=1, pos -= rmapW-1
	jump_north = rmap: jump_east(M1), (x,y-r)
	jump_west = map: jump_west(M0), (x-r,y)

	SW = jump_intercardinal_neg(M0=rmap,M1=map)
	map:  x-=1, y+=1, pos += mapW-1
	rmap: x-=1, y-=1, pos -= rmapW+1
	jump_south = rmap: jump_west(M0), (x,y+r)
	jump_west = map: jump_west(M1), (x-r,y)
	*/

	union LongJumpRes {
		uint32_t dist[2];
		uint64_t joint;
	};
	struct Walker
	{
		const gridmap* map[2];
		uint32_t node_at[2];
		int32_t adj_width[2];
		union {
			uint8_t row[2];
			uint16_t row_i;
		};
		uint32_t goal[2];

		void next_index() noexcept
		{
			node_at[0] = static_cast<uint32_t>(static_cast<int32_t>(node_at[0]) + adj_width[0]);
			node_at[1] = static_cast<uint32_t>(static_cast<int32_t>(node_at[1]) + adj_width[1]);
		}

		uint8_t get_row() const noexcept
		{
			// get node_at-1..node_at+1
			return static_cast<uint8_t>(map[0]->template get_span<3>(pad_id{node_at[0]-1}));
		}
		void first_row() noexcept
		{
			row[1] = get_row();
		}
		void next_row() noexcept
		{
			next_index();
			row[0] = row[1];
			row[1] = get_row();
		}
		// jps_id get_last_row() const noexcept
		// {
		// 	return jps_id{static_cast<uint32_t>(static_cast<int32_t>(node_at[0]) - adj_width[0])};
		// }
		// jps_rid get_last_rrow() const noexcept
		// {
		// 	return jps_rid{static_cast<uint32_t>(static_cast<int32_t>(node_at[1]) - adj_width[1])};
		// }
		jps_id adj_hori(uint32_t node, uint32_t dist) const noexcept
		{
			if constexpr (D == NORTHEAST || D == SOUTHEAST) {
				return jps_id{node + dist};
			} else {
				return jps_id{node - dist};
			}
		}
		jps_id adj_vert(uint32_t node, uint32_t dist) const noexcept
		{
			if constexpr (D == NORTHEAST || D == SOUTHEAST) {
				return jps_id{node + (adj_width[0]-1) * dist};
			} else {
				return jps_id{node + (adj_width[0]+1) * dist};
			}
		}
		bool valid_space() const noexcept
		{
			// east | west differernce
			// north/south does not make a difference
			if constexpr (D == NORTHEAST || D == SOUTHEAST) {
				// we want from grid
				// .xx  == row[0] = 0bxx.
				//  xx. == row[1] = 0b.xx
				// all[x] = 1
				constexpr uint16_t mask = std::endian::native == std::endian::little ?
					0b0000'0011'0000'0110 :
					0b0000'0110'0000'0011;
				return (row_i & mask) == mask;
			} else {
				// we want from grid
				//  xx. == row[0] = 0b.xx
				// .xx  == row[1] = 0bxx.
				// all[x] = 1
				constexpr uint16_t mask = std::endian::native == std::endian::little ?
					0b0000'0110'0000'0011 :
					0b0000'0011'0000'0110;
				return (row_i & mask) == mask;
			}
		}
		// return {hori,vert}
		LongJumpRes long_jump()
		{
			if constexpr (D == NORTHEAST) {
				return {
					jump_east(*map[0], node_at[0], goal[0]), // east
					jump_east(*map[1], node_at[1], goal[1]) // north
				};
			} else if constexpr (D == SOUTHEAST) {
				return {
					jump_east(*map[0], node_at[0], goal[0]), // east
					jump_west(*map[1], node_at[1], goal[1]) // south
				};
			} else if constexpr (D == SOUTHWEST) {
				return {
					jump_west(*map[0], node_at[0], goal[0]), // west
					jump_west(*map[1], node_at[1], goal[1]) // south
				};
			} else if constexpr (D == NORTHWEST) {
				return {
					jump_west(*map[0], node_at[0], goal[0]), // west
					jump_east(*map[1], node_at[1], goal[1]) // north
				};
			}
		}
	} walker;
	walker.map[0] = map_;
	walker.map[1] = rmap_.get();
	walker.node_at[0] = static_cast<uint32_t>(node);
	walker.node_at[1] = static_cast<uint32_t>(rnode);
	// 0 = map, 1 = rname
	// setup node and rnode adjust per diagonal
	switch (D) {
	case NORTHEAST:
		walker.adj_width[0] = -static_cast<int32_t>(map_width_ - 1); // - (mapW-1)
		walker.adj_width[1] = static_cast<int32_t>(rmap_width_ + 1); // + (rmapW+1)
		break;
	case SOUTHEAST:
		walker.adj_width[0] = static_cast<int32_t>(map_width_ + 1); // + (mapW+1)
		walker.adj_width[1] = static_cast<int32_t>(rmap_width_ - 1); // + (rmapW-1)
		break;
	case SOUTHWEST:
		walker.adj_width[0] = static_cast<int32_t>(map_width_ - 1); // + (mapW-1)
		walker.adj_width[1] = -static_cast<int32_t>(rmap_width_ + 1); // - (rmapW+1)
		break;
	case NORTHWEST:
		walker.adj_width[0] = -static_cast<int32_t>(map_width_ + 1); // - (mapW+1)
		walker.adj_width[1] = -static_cast<int32_t>(rmap_width_ - 1); // - (rmapW-1)
		break;
	}
	walker.goal[0] = goal_.id;
	walker.goal[1] = rgoal_.id;

	assert(!(feature_prune_intercardinal() || feature_store_cardinal()) // both not enabled = fine
		|| (result_node != nullptr && result_cost != nullptr) // must be set if enabled
	);

	if constexpr (!feature_prune_intercardinal()) {
		uint32_t walk_count = 1;
		walker.first_row();
		while (true) {
			walker.next_row();
			if (!walker.valid_space())
				return {jps_id::none(), jps_rid::none(), 0};
			if (walker.node_at[0] == walker.goal[0]) [[unlikely]] {
				// reached goal
				intercardinal_jump_result result;
				result.node = jps_id{walker.node_at[0]};
				result.rnode = jps_rid::none(); // walker.get_last_rrow();
				result.dist = walk_count;
				if (feature_store_cardinal()) {
					result_node[0] = jps_id::none();
					result_node[1] = jps_id::none();
				}
				return result;
			}
			auto res = walker.long_jump();
			if (res.joint != 0) {
				intercardinal_jump_result result;
				result.node = jps_id{walker.node_at[0]};
				result.rnode = jps_rid::none(); // walker.get_last_rrow();
				result.dist = walk_count;
				if (feature_store_cardinal()) {
					cost_t current_cost = walk_count * warthog::DBL_ROOT_TWO;
					if (res.dist[0] != 0) {
						result_node[0] = walker.adj_hori(result.node.id, res.dist[0]);
						result_cost[0] = current_cost + res.dist[0] * warthog::DBL_ONE;
					} else {
						result_node[0] = jps_id::none();
					}
					if (res.dist[1] != 0) {
						result_node[1] = walker.adj_vert(result.node.id, res.dist[1]);
						result_cost[1] = current_cost + res.dist[1] * warthog::DBL_ONE;
					} else {
						result_node[1] = jps_id::none();
					}
				}
				// found jump point
				return result;
			}
			walk_count += 1;
		}
	} else {
		// prunes intercardinal, progress and add successors to count
		assert(result_size > 2);
		result_size -= 1; // ensure there is always space for at least 2 results
		uint32_t walk_count = 1;
		uint32_t result_count = 0;
		walker.first_row();
		while (result_count < result_size) {
			walker.next_row();
			if (!walker.valid_space())
				return {jps_id::none(), jps_rid::none(), result_count};
			if (walker.node_at[0] == walker.goal[0]) [[unlikely]] {
				// reached goal
				intercardinal_jump_result result;
				result.node = jps_id{walker.node_at[0]};
				result.rnode = jps_rid::none(); // walker.get_last_rrow();
				result.dist = 0;
				return result;
			}
			auto res = walker.long_jump();
			cost_t current_cost = walk_count * warthog::DBL_ROOT_TWO;
			if (res.dist[0] != 0) { // east/west
				result_count += 1;
				*result_node++ = walker.adj_hori(walker.node_at[0], res.dist[0]);
				*result_cost++ = current_cost + res.dist[0] * warthog::DBL_ONE;
			}
			if (res.dist[1] != 0) { // north/south
				result_count += 1;
				// NORTH/SOUTH handles the correct sing, adjust for EAST/WEST diff
				*result_node++ = walker.adj_vert(walker.node_at[0], res.dist[1]);
				*result_cost++ = current_cost + res.dist[1] * warthog::DBL_ONE;
			}
			walk_count += 1;
		}
		// not enough buffer, return result and start again
		return {jps_id{walker.node_at[0]}, jps_rid{walker.node_at[1]}, result_count};
	}
}

}

#endif // JPS_JUMP_JUMP_POINT_ONLINE_H
