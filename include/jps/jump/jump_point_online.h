#ifndef JPS_JUMP_JUMP_POINT_ONLINE_H
#define JPS_JUMP_JUMP_POINT_ONLINE_H

//
// jps/jump/jump_point_offline.h
//
// Online jump-point locator.
// Contains utility functions for online jump-point locators
//
// The table will precompute the jump-point distances from an online locator
// and store them into a jump_point_table.
//
// @author dharabor & Ryan Hechenberger
// @created 2025-11-20
//

#include "block_online.h"
#include "jump.h"
#include <array>
#include <jps/domain/rotate_gridmap.h>
#include <jps/forward.h>
#include <memory>
#include <warthog/util/template.h>

namespace jps::jump
{

class jump_point_online
{
public:
	using bittable    = ::warthog::domain::gridmap::bittable;
	using rotate_grid = domain::gridmap_rotate_table_convs;

	jump_point_online() = default;
	jump_point_online(const rotate_grid& map) { set_map(map); }
	~jump_point_online() = default;

	void
	set_map(const rotate_grid& map);
	// void
	// set_target(jps_id target_id) noexcept;
	// void
	// set_target(point target_id) noexcept;

	// /// @brief avoid modifying these grids accidentally
	// bittable
	// map() const noexcept
	// {
	// 	return map_;
	// }
	// /// @brief care should be taken to avoid modifying these grids
	// bittable
	// rmap() noexcept
	// {
	// 	return rmap_;
	// }

	/// @brief same as jump_cardinal_next(point) but is given the correct
	/// grid_id type
	/// @tparam D cardinal direction_id (NORTH_ID,SOUTH_ID,EAST_ID,WEST_ID)
	/// @param node_id the id pairs for grid and rgrid (at loc)
	/// @return >0: jump point n steps away, <=0: blocker -n steps away
	template<direction_id D>
	    requires CardinalId<D>
	jump_distance
	jump_cardinal_next(domain::grid_pair_id node_id);

	/// @brief returns all intercardinal jump points up to max_distance
	/// (default inf)
	///        and max of results_size
	/// @tparam D intercardinal direction_id
	/// (NORTHEAST_ID,NORTHWEST_ID,SOUTHEAST_ID,SOUTHWEST_ID)
	/// @param node_id the id pairs for grid and rgrid
	/// @param result_size maximum number of results that can fit in result
	/// @param result pointer to results storage of at least size result_size
	/// @param max_distance the maximum intercardinal distance to scan to
	/// @return pair first: number of results returned. second: the end
	/// distance
	///
	/// This function is designed to efficiently find all jump points
	/// intercardinally. The returned point is either on the final result, the
	/// max_distance location (loc + max_distance) or the point in-front of the
	/// blocker(deadend). This function is intended to be run multiple times by
	/// passing the return loc into the next row, until either reaching a
	/// deadend or some algorithmic specific stopping criteria.
	template<direction_id D>
	    requires InterCardinalId<D>
	std::pair<uint16_t, jump_distance>
	jump_intercardinal_many(
	    domain::grid_pair_id node_id, intercardinal_jump_result* result,
	    uint16_t result_size,
	    jump_distance max_distance
	    = std::numeric_limits<jump_distance>::max());

	/// @brief shoot ray to target point
	/// @param node_id the id pairs for grid and rgrid (at loc)
	/// @param loc shoot from loc
	/// @param target shoot to target
	/// @return pair <intercardinal-distance, cardinal-distance>, if both >= 0
	/// then target is visible,
	///         first<0 means intercardinal reaches blocker at -first distance
	///         (second will be -1) first>=0 second<0 means cardinal blocker at
	///         -second distance away second<0 mean target is blocked in
	///         general
	std::pair<jump_distance, jump_distance>
	jump_target(domain::grid_pair_id node_id, point loc, point target);

	size_t
	mem()
	{
		return sizeof(this);
	}

protected:
	static int32_t
	jump_east(bittable map, uint32_t node)
	{
		return jump_point_online_hori<true, false>(map, node);
	}
	static int32_t
	jump_west(bittable map, uint32_t node)
	{
		return jump_point_online_hori<false, false>(map, node);
	}

protected:
	rotate_grid map_;
};

void
jump_point_online::set_map(const rotate_grid& orig)
{
	map_ = orig;
}

template<direction_id D>
    requires CardinalId<D>
jump_distance
jump_point_online::jump_cardinal_next(domain::grid_pair_id node_id)
{
	if constexpr(D == NORTH_ID || D == EAST_ID)
	{
		return jump_east(
		    map_.table(domain::rgrid_index<D>),
		    static_cast<uint32_t>(get_d<D>(node_id)));
	}
	else if constexpr(D == SOUTH_ID || D == WEST_ID)
	{
		return jump_west(
		    map_.table(domain::rgrid_index<D>),
		    static_cast<uint32_t>(get_d<D>(node_id)));
	}
	else
	{
		assert(false);
		return 0;
	}
}

template<direction_id D>
    requires InterCardinalId<D>
std::pair<uint16_t, jump_distance>
jump_point_online::jump_intercardinal_many(
    domain::grid_pair_id node_id, intercardinal_jump_result* result,
    uint16_t result_size, jump_distance max_distance)
{
	assert(result_size > 0);
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

	IntercardinalWalker<D> walker; // class to walk
	// setup the walker members
	// 0 = map, 1 = rmap
	walker.map = {map_[0], map_[1]};
	walker.map_width(map_.width());
	walker.rmap_width(map_.rwidth());
	walker.node_at[0] = static_cast<uint32_t>(get<grid_id>(node_id));
	walker.node_at[1] = static_cast<uint32_t>(get<rgrid_id>(node_id));
	assert(
	    map_.map().get(grid_id(walker.node_at[0]))
	    && map_.rmap().get(grid_id(walker.node_at[1])));

	// JPS, stop at the first intercardinal turning point
	uint16_t results_count   = 0;
	jump_distance walk_count = 0;
	walker.first_row();
	while(walk_count < max_distance) // if equal, max distance is reached
	{
		walker.next_row(); // walk_count adjusted at end of loop
		if(!walker.valid_row()) [[unlikely]] // no successors
			return {results_count, static_cast<jump_distance>(-walk_count)};
		walk_count += 1;
		//
		// handle hori/vert long jump
		//
		auto res = walker.long_jump();
		if(res)
		{ // at least one jump has a turning point
			assert(results_count < result_size);
			*(result++) = {walk_count, res.dist[0], res.dist[1]};
			if(++results_count == result_size) break;
		}
	}

	return {results_count, walk_count};
}

std::pair<jump_distance, jump_distance>
jump_point_online::jump_target(
    domain::grid_pair_id node_id, point loc, point target)
{
	// direction_id real_d = d != 255 ? static_cast<direction_id>(d) :
	// warthog::grid::point_to_direction_id(loc, target);
	auto [xd, yd] = warthog::grid::point_signed_diff(loc, target);
	jump_distance inter_len;
	jump_distance cardinal_len;
	direction_id d;
	if(xd != 0 && yd != 0)
	{
		// diagonal
		BasicIntercardinalWalker walker;
		if(xd > 0)
		{ // east
			if(yd > 0)
			{ // south
				d = SOUTHEAST_ID;
				walker.set_map<SOUTHEAST_ID>(map_[0], map_.width());
			}
			else
			{
				d = NORTHEAST_ID;
				walker.set_map<NORTHEAST_ID>(map_[0], map_.width());
			}
		}
		else
		{ // west
			if(yd > 0)
			{ // south
				d = SOUTHWEST_ID;
				walker.set_map<SOUTHWEST_ID>(map_[0], map_.width());
			}
			else
			{
				d = NORTHWEST_ID;
				walker.set_map<NORTHWEST_ID>(map_[0], map_.width());
			}
		}
		walker.node_at = static_cast<uint32_t>(get<grid_id>(node_id));
		inter_len
		    = static_cast<jump_distance>(std::min(std::abs(xd), std::abs(yd)));
		walker.first_row();
		for(jump_distance i = 0; i < inter_len; ++i)
		{
			walker.next_row();
			if(!walker.valid_row())
			{
				// diagonal blocked before reaching turn, report
				return {static_cast<jump_distance>(-i), -1};
			}
		}
		// update loc
		spoint dia_unit = dir_unit_point(d);
		dia_unit.x     *= inter_len;
		dia_unit.y     *= inter_len;
		loc             = loc + dia_unit;
		xd             -= dia_unit.x;
		yd             -= dia_unit.y;
	}
	else
	{
		// no diagonal
		inter_len = 0;
	}
	assert(xd == 0 || yd == 0);
	if(yd == 0)
	{
		// horizontal ray
		if(xd == 0) [[unlikely]]
		{
			// at target
			return {inter_len, 0};
		}
		else if(xd > 0)
		{
			// east
			cardinal_len
			    = jump_point_online_hori_target<domain::rgrid_east<EAST_ID>>(
			        map_.table(domain::rgrid_index<EAST_ID>),
			        static_cast<uint32_t>(map_.point_to_id_d<EAST_ID>(loc)),
			        static_cast<uint32_t>(
			            map_.point_to_id_d<EAST_ID>(target)));
		}
		else
		{
			// west
			cardinal_len
			    = jump_point_online_hori_target<domain::rgrid_east<WEST_ID>>(
			        map_.table(domain::rgrid_index<WEST_ID>),
			        static_cast<uint32_t>(map_.point_to_id_d<WEST_ID>(loc)),
			        static_cast<uint32_t>(
			            map_.point_to_id_d<WEST_ID>(target)));
		}
	}
	else if(yd > 0)
	{
		// south
		cardinal_len
		    = jump_point_online_hori_target<domain::rgrid_east<SOUTH_ID>>(
		        map_.table(domain::rgrid_index<SOUTH_ID>),
		        static_cast<uint32_t>(map_.point_to_id_d<SOUTH_ID>(loc)),
		        static_cast<uint32_t>(map_.point_to_id_d<SOUTH_ID>(target)));
	}
	else
	{
		// north
		cardinal_len
		    = jump_point_online_hori_target<domain::rgrid_east<NORTH_ID>>(
		        map_.table(domain::rgrid_index<NORTH_ID>),
		        static_cast<uint32_t>(map_.point_to_id_d<NORTH_ID>(loc)),
		        static_cast<uint32_t>(map_.point_to_id_d<NORTH_ID>(target)));
	}
	if(cardinal_len == 0) // corner case, do not return {>0,0} value at
	                      // blocker, instead return {<,-1}
		return {-inter_len, -1};
	else
		return {inter_len, cardinal_len};
}

} // namespace jps::jump

#endif // JPS_JUMP_JUMP_POINT_ONLINE_H
