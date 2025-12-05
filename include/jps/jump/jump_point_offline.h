#ifndef JPS_JUMP_JUMP_POINT_OFFLINE_H
#define JPS_JUMP_JUMP_POINT_OFFLINE_H

//
// jps/jump/jump_point_offline.h
//
// Offline level jump-point locator.
// Contains two components, the jump_point_table which handles how jump-points
// are stored in all 8-directions.
// The jump_point_offline is given a templated jump_point_table and an online
// jump-point locator.
//
// The table will precompute the jump-point distances from an online locator
// and store them into a jump_point_table.
//
// @author dharabor & Ryan Hechenberger
// @created 2025-11-20
//

#include "jump_point_online.h"
#include <array>
#include <charconv>
#include <ostream>
#include <warthog/util/template.h>

namespace jps::jump
{

/// @brief Store, set and access offline jump-point results in all
/// 8-directions.
/// @tparam ChainJump if true: store jumps as 1-byte, chaining for long jumps;
///         false: stores in 2-bytes the full jump distance
/// @tparam DeadEnd if true: track deadend as negative, false: deadend not
/// recorded
template<bool ChainJump = false, bool DeadEnd = true>
struct jump_point_table
{
	using direction_id   = warthog::grid::direction_id;
	using jump_res_width = std::conditional_t<ChainJump, uint8_t, uint16_t>;
	using jump_res       = std::conditional_t<
	          DeadEnd, std::make_signed_t<jump_res_width>, jump_res_width>;
	using length = int32_t;
	struct alignas(int64_t) cell : std::array<jump_res, 8>
	{ };

	static consteval bool
	chain_jump() noexcept
	{
		return ChainJump;
	}
	static consteval bool
	dead_end() noexcept
	{
		return DeadEnd;
	}

	std::unique_ptr<cell[]> db;
	uint32_t width = 0;
	uint32_t cells = 0;
	static constexpr uint32_t
	node_count(uint32_t width, uint32_t height) noexcept
	{
		return static_cast<size_t>(width) * static_cast<size_t>(height);
	}
	constexpr size_t
	mem() noexcept
	{
		return cells * sizeof(jump_res);
	}

	/// @return the value that indicates to chain
	static consteval jump_res
	chain_value() noexcept
	{
		return DeadEnd ? std::numeric_limits<jump_res>::min()
		               : std::numeric_limits<jump_res>::max();
	}
	/// @return the length to jump after chain_value
	static consteval length
	chain_stride() noexcept
	{
		constexpr length acv = std::abs(static_cast<length>(chain_value()));
		return acv - 2; // direct jump max at chain_stride()+1, but chain only
		                // chain_stride() as we must never reach 0
	}
	static_assert(
	    !chain_jump()
	        || std::abs(int32_t(std::make_signed_t<jump_res>(chain_value())))
	            > int32_t(chain_stride()),
	    "absolute value of chain_value() must be greater than chain_length()");

	/// @brief setup db, init to zero
	void
	init(uint32_t width, uint32_t height)
	{
		this->cells = node_count(width, height);
		this->width = width;
		this->db    = std::make_unique<cell[]>(cells);
	}
	/// @brief sets precomputed jump-point along line [loc...loc+len)
	///        when len reaches 0, final cell is not set
	/// @param d direction of line
	/// @param loc location
	/// @param len length to cover, exclusive end
	void
	set_line(direction_id d, grid_id loc, length len) noexcept
	{
		// no negative for deadend
		assert(d < 8);
		assert(db != nullptr);
		assert(loc.id < cells);
		assert(DeadEnd || len >= 0);
		assert(std::abs(len) < std::numeric_limits<int16_t>::max());
		uint32_t id           = loc.id;
		const uint32_t id_adj = warthog::grid::dir_id_adj(d, width);
		const length len_adj
		    = DeadEnd ? static_cast<length>(len >= 0 ? -1 : 1) : -1;
		while(len != 0)
		{
			jump_res value;
			if constexpr(!ChainJump)
			{
				// just use value
				value = static_cast<jump_res>(len);
			}
			else
			{
				// if len <= chain_length(), use len, otherwise use
				// chain_value() to force a chain lookup
				if constexpr(DeadEnd)
				{
					// seperate DeadEnd code to remove abs
					static_assert(std::is_signed_v<jump_res>);
					value = std::abs(len) <= chain_stride() + 1
					    ? static_cast<jump_res>(len)
					    : chain_value();
				}
				else
				{
					value = len <= chain_stride() + 1
					    ? static_cast<jump_res>(len)
					    : chain_value();
				}
			}
			db[id][d] = value;
			id       += id_adj;
			len      += len_adj;
		}
	}
	/// @brief get pre-computed jump at location in direction
	/// @param d direction
	/// @param loc location
	/// @return jump from loc in d
	length
	get_jump(direction_id d, grid_id loc) noexcept
	{
		// no negative for deadend
		assert(d < 8);
		assert(db != nullptr);
		assert(loc.id < cells);
		if constexpr(!ChainJump) { return static_cast<length>(db[loc.id][d]); }
		else
		{
			jump_res u = static_cast<length>(db[loc.id][d]);
			if(u != chain_value()) { return static_cast<length>(u); }
			else { return chain_jump(d, loc); }
		}
	}
	/// @brief get mutable cell
	/// @param loc location
	/// @return cell entry for loc
	cell&
	get(grid_id loc) noexcept
	{
		assert(db != nullptr);
		assert(loc.id < cells);
		return db[loc.id];
	}
	/// @brief get immutable cell
	/// @param loc location
	/// @return cell entry for loc
	cell
	get(grid_id loc) const noexcept
	{
		assert(db != nullptr);
		assert(loc.id < cells);
		return db[loc.id];
	}
	/// @brief perform a chain jump, assumes loc is chain value
	///        assumes user manually checked db value at loc and value was
	///        chain_value()
	/// @param d direction of jump
	/// @param loc location
	/// @return jump length, at least chain_length()+1
	/// @pre db[loc.id][d] == chain_value(), loc must be chained
	length
	chain_jump(direction_id d, grid_id loc) noexcept
	    requires(ChainJump)
	{
		assert(db != nullptr);
		assert(loc.id < cells);
		assert(db[loc.id][d] == chain_value());
		const uint32_t id_adj = static_cast<uint32_t>(chain_stride())
		    * warthog::grid::dir_id_adj(d, width);
		length len = 0;
		jump_res j;
		do
		{
			// continue from previous jump
			loc.id += id_adj;
			len    += chain_stride();
			j       = static_cast<length>(db[loc.id][d]);
		} while(j == chain_value());
		assert(j != 0);
		if constexpr(DeadEnd)
		{
			// check if block and negate, j is signed in this version
			len = j >= 0 ? (len + j) : -(len - j);
		}
		else
		{
			// j is unsigned
			len += static_cast<std::make_unsigned_t<length>>(j);
		}
		return len;
	}

	/// @brief get jump cell
	/// @param loc location
	/// @return cell
	cell
	operator[](grid_id loc) const noexcept
	{
		assert(db != nullptr);
		assert(loc.id < cells);
		return db[loc.id];
	}

	std::ostream&
	print(std::ostream& out)
	{
		std::array<char, 8 * 8> buffer;
		for(uint32_t i = 0; i < cells;)
		{
			for(uint32_t j = 0; j < width; ++j, ++i)
			{
				// fill buffer with cell
				char *at = buffer.begin(), *end = buffer.end();
				for(int k = 0; k < 8; ++k)
				{
					jump_distance dist
					    = get_jump(static_cast<direction_id>(k), grid_id{i});
					auto res = std::to_chars(at, end, dist);
					if(res.ec != std::errc{})
					{
						// error, exit
						out.setstate(std::ios::failbit);
						return out;
					}
					if(at == end)
					{
						// out of space, should not happen
						out.setstate(std::ios::failbit);
						return out;
					}
					*at++ = k + 1 < 8 ? ',' : '\n';
				}
				out << buffer.data() << (j + 1 < width ? '\t' : '\n');
			}
		}
	}
};

template<
    typename JumpTable   = jump_point_table<>,
    typename OnlinePoint = jump_point_online>
class jump_point_offline : public OnlinePoint
{
public:
	using jump_point_online::jump_point_online;
	using typename OnlinePoint::bittable;
	using typename OnlinePoint::rotate_grid;

	template<direction_id D>
	    requires CardinalId<D>
	jump_distance
	jump_cardinal_next(domain::grid_pair_id node_id)
	{
		return static_cast<jump_distance>(
		    jump_table_.get_jump(D, get<grid_id>(node_id)));
	}

	template<direction_id D>
	    requires InterCardinalId<D>
	std::pair<uint16_t, jump_distance>
	jump_intercardinal_many(
	    domain::grid_pair_id node_id, intercardinal_jump_result* result,
	    uint16_t result_size,
	    jump_distance max_distance = std::numeric_limits<jump_distance>::max())
	{
		constexpr direction_id Dhori = dir_intercardinal_hori(D);
		constexpr direction_id Dvert = dir_intercardinal_vert(D);
		grid_id node                 = get<grid_id>(node_id);
		const uint32_t node_adj      = dir_id_adj(D, this->map_.width());
		std::pair<uint16_t, jump_distance> res{0, 0};
		for(/*res.first*/; res.first < result_size;)
		{
			jump_distance dist
			    = static_cast<jump_distance>(jump_table_.get_jump(D, node));
			if(dist <= 0)
			{
				res.second = static_cast<jump_distance>(-res.second + dist);
				break;
			}
			res.first  += 1;
			res.second += dist;
			node.id    += dist * node_adj;
			// found point
			intercardinal_jump_result resi;
			resi.inter = res.second;
			resi.hori  = static_cast<jump_distance>(
                jump_table_.get_jump(Dhori, node));
			resi.vert = static_cast<jump_distance>(
			    jump_table_.get_jump(Dvert, node));
			*(result++) = resi;
			if(res.second > max_distance) break;
		}
		return res;
	}

	/// @brief test jump directly to target point is visible or blocked
	///        i.e. has line-of-sight or is blocked and where
	/// @param node_id the id pairs for grid and rgrid (at loc)
	/// @param loc x/y location (node_id points here)
	/// @param target target point to check visiblity to
	/// @return pair <intercardinal-distance, cardinal-distance>, if both >= 0
	/// then target is visible,
	///         first<0 means intercardinal reaches blocker at -first distance
	///         (second will be -1) first>=0 second<0 means cardinal blocker at
	///         -second distance away second<0 mean target is blocked in
	///         general
	std::pair<jump_distance, jump_distance>
	jump_target(domain::grid_pair_id node_id, point loc, point target)
	{
		if constexpr(!JumpTable::dead_end())
		{
			// dead end locations are not stored, so may not be able to find
			// target, use online version
			return OnlinePoint::jump_target(node_id, loc, target);
		}
		else
		{
			// direction_id real_d = d != 255 ? static_cast<direction_id>(d) :
			// warthog::grid::point_to_direction_id(loc, target);
			if(loc == target) return {0, 0};
			jump_distance inter_len    = 0;
			jump_distance cardinal_len = 0;
			struct
			{
				uint32_t v;
				uint32_t adj;
			} id;
			id.v           = get<grid_id>(node_id).id;
			direction_id d = point_to_direction_id(loc, target);
			id.adj         = dir_id_adj(d, this->map_.width());
			auto [xd, yd]  = warthog::grid::point_signed_diff(
                loc, target); // outside for cardinal pass
			jump_distance jump_to;
			if(is_intercardinal_id(d))
			{
				jump_to = static_cast<jump_distance>(
				    std::min(std::abs(xd), std::abs(yd)));
				uint32_t startv = id.v;
				do
				{
					jump_distance dist = static_cast<jump_distance>(
					    jump_table_.get_jump(d, grid_id(id.v)));
					if(dist <= 0)
					{
						// hit wall, final check
						inter_len += -dist;
						id.v      += to_unsigned_jump_distance(-dist) * id.adj;
						if(inter_len < jump_to) // failed to reach target
							return {-inter_len, -1};
						break;
					}
					inter_len += dist;
					id.v      += to_unsigned_jump_distance(dist) * id.adj;
				} while(inter_len < jump_to);
				inter_len = jump_to;
				id.v      = startv
				    + jump_to * id.adj; // correct for over jumping jump_to
				if(std::abs(xd) > std::abs(yd))
				{
					// east/west
					jump_to
					    = static_cast<jump_distance>(std::abs(xd) - jump_to);
					d = xd > 0 ? EAST_ID : WEST_ID;
				}
				else if(std::abs(xd) < std::abs(yd))
				{
					// noth/south
					jump_to
					    = static_cast<jump_distance>(std::abs(yd) - jump_to);
					d = yd > 0 ? SOUTH_ID : NORTH_ID;
				}
				else
				{
					// target reached
					return {inter_len, 0};
				}
				id.adj = dir_id_adj(d, this->map_.width());
			}
			else
			{
				// target is cardinal to loc, max is correct jump_to
				jump_to = static_cast<jump_distance>(
				    std::max(std::abs(xd), std::abs(yd)));
			}
			// jump_to and d are correct for the cardinal jump
			do
			{
				jump_distance dist = static_cast<jump_distance>(
				    jump_table_.get_jump(d, grid_id(id.v)));
				if(dist <= 0)
				{
					// hit wall, final check
					cardinal_len += -dist;
					id.v         += to_unsigned_jump_distance(-dist) * id.adj;
					if(cardinal_len < jump_to)
					{                         // failed to reach target
						if(cardinal_len == 0) // corner case
							return {-inter_len, -1};
						else
							return {inter_len, -cardinal_len};
					}
					break;
				}
				cardinal_len += dist;
				id.v         += to_unsigned_jump_distance(dist) * id.adj;
			} while(cardinal_len < jump_to);
			return {inter_len, jump_to};
		}
	}

	/// @brief set the underlying map, re-computing offline jump table
	void
	set_map(const rotate_grid& map)
	{
		OnlinePoint::set_map(map);
		// compute offline jump-point table
		compute_jump_table();
	}
	/// @brief computes the jump table. Linear complexity to size of grid O(WH)
	void
	compute_jump_table()
	{
		const uint32_t width  = this->map_.width();
		const uint32_t height = this->map_.height();
		jump_table_.init(width, height);
		auto&& point_in_range
		    = [=](point p) noexcept { return p.x < width && p.y < height; };

		// handle cardinal scans

		// compute jump table in N,S,E,W
		// preset how to scan in each direction to reduce code to a for-loop
		struct CardinalScan
		{
			direction_id d;
			point start;
			spoint row_adj;
		};
		const std::array<CardinalScan, 4> scans{
		    {{NORTH_ID, point(0, height - 1), spoint(1, 0)},
		     {SOUTH_ID, point(0, 0), spoint(1, 0)},
		     {EAST_ID, point(0, 0), spoint(0, 1)},
		     {WEST_ID, point(width - 1, 0), spoint(0, 1)}}};

		// esseintally compile-type for-each N,S,E,W
		// computes each direction and stores result in table in linear time on
		// grid size
		using jump_cardinal_type = jump_distance(OnlinePoint*, uint32_t);
		warthog::util::for_each_integer_sequence<std::integer_sequence<
		    direction_id, NORTH_ID, EAST_ID, SOUTH_ID, WEST_ID>>([&](auto iv) {
			constexpr direction_id di = decltype(iv)::value;
			constexpr int map_id      = domain::rgrid_index<di>;
			const auto map            = this->map_[map_id];
			CardinalScan s
			    = *std::find_if(scans.begin(), scans.end(), [di](auto& s) {
				      return s.d == di;
			      });
			// start scan
			point node       = s.start;
			const spoint adj = dir_unit_point(s.d);
			// for each directional row
			for(point node = s.start; point_in_range(node);
			    node       = node + s.row_adj)
			{
				point current_node = node;
				// current_node loops column
				while(point_in_range(current_node))
				{
					auto current_id
					    = this->map_.point_to_pair_id(current_node);
					if(map.get(grid_id(get<map_id>(current_id))))
					{
						jump_distance d
						    = OnlinePoint::template jump_cardinal_next<di>(
						        current_id);
						if(d == 0) [[unlikely]]
						{
							// immediently blocked
							// jump_table_.get(row_node)[di] = 0;
							current_node = current_node
							    + adj; // we know the next cell is a blocker
							// assert(!this->map_.map().get(grid_id{node.id +
							// col_adj}));
						}
						else
						{
							// store result
							jump_table_.set_line(
							    di, get<grid_id>(current_id), d);
							current_node = current_node
							    + std::abs(d)
							        * adj; // next cell is the reached cell
							assert(point_in_range(
							    current_node)); // j should never jump past
							                    // edge
							// assert(this->map_.map().get(grid_id{node.id + j
							// * col_adj}));
						}
					}
					else
					{
						current_node = current_node
						    + adj; // is invalid cell, check the next
					}
				}
			}
		});

		//
		// InterCardinal scans
		//

		// compute jump table in NE,NW,SE,SW
		// preset how to scan in each direction to reduce code to a for-loop
		struct ICardinalScan
		{
			direction_id d;
			point start; // start location
		};
		const std::array<ICardinalScan, 4> Iscans{
		    {{NORTHEAST_ID, point(0, height - 1)},
		     {NORTHWEST_ID, point(width - 1, height - 1)},
		     {SOUTHEAST_ID, point(0, 0)},
		     {SOUTHWEST_ID, point(width - 1, 0)}}};

		for(auto s : Iscans)
		{
			const direction_id dh = dir_intercardinal_hori(s.d);
			const direction_id dv = dir_intercardinal_vert(s.d);
			const spoint adj      = dir_unit_point(s.d);
			BasicIntercardinalWalker walker;
			walker.set_map(s.d, this->map_.map(), this->map_.width());
			for(int axis = 0; axis < 2; ++axis)
			{
				// 0 = follow hori edge, then follow vert edge
				point start = s.start;
				if(axis == 0) // adjust 1 axis to get diagonal along the start
					start.x -= static_cast<uint32_t>(adj.x);
				while(true)
				{ // start location
					if(axis == 0)
						start.x += static_cast<uint32_t>(adj.x);
					else
						start.y += static_cast<uint32_t>(adj.y);
					if(!point_in_range(start))
						break; // out of bounds, do nothing
					// now scan along the diagonal
					point loc = start;
					while(true)
					{
						if(!point_in_range(loc)) break; // out of bounds, end
						if(!this->map_.map().get(this->map_.point_to_id(loc)))
						{
							// blocker
							loc = loc + adj;
							continue;
						}
						// calc distance
						point currentloc   = loc;
						jump_distance dist = 0;
						// walk from current loc
						walker.node_at = static_cast<uint32_t>(
						    this->map_.point_to_id(currentloc));
						walker.first_row();
						while(true)
						{
							point nextloc = currentloc + adj;
							bool valid    = point_in_range(nextloc);
							if(valid)
							{
								walker.next_row();
								valid = walker.valid_row();
							}
							// loc in grid and non-corner cutting dia is clear
							if(valid)
							{
								// traversable tile
								dist += 1;
								const auto& cell
								    = jump_table_[this->map_.point_to_id(
								        nextloc)];
								if(auto id = this->map_.point_to_id(nextloc);
								   jump_table_.get_jump(dh, id) > 0
								   || jump_table_.get_jump(dv, id) > 0)
								{
									// turning point here
									jump_table_.set_line(
									    s.d, this->map_.point_to_id(loc),
									    dist);
									loc = nextloc;
									break; // done with this line
								}
							}
							else
							{
								// reached blocker or edge of map
								if(dist != 0)
								{
									jump_table_.set_line(
									    s.d, this->map_.point_to_id(loc),
									    -dist);
								}
								loc = nextloc;
								break; // done with this line
							}
							currentloc = nextloc;
						}
					}
				}
			}
		}
	}

protected:
	JumpTable jump_table_;
};

} // namespace jps::jump

#endif // JPS_JUMP_JUMP_POINT_OFFLINE_H
