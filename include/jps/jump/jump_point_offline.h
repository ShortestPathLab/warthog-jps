#ifndef JPS_JUMP_JUMP_POINT_OFFLINE_H
#define JPS_JUMP_JUMP_POINT_OFFLINE_H

// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011,
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//

#include "jump_point_online.h"
#include <warthog/util/template.h>
#include <array>
#include <ostream>
#include <charconv>

namespace jps::jump
{

/// @brief Store, set and access offline jump-point results
/// @tparam ChainJump if true: store 1-byte jumps that chain; false: 2-byte full jump
/// @tparam DeadEnd if true: track deadend as negative, false: only jump-points
template <bool ChainJump = false, bool DeadEnd = true>
struct jump_point_table
{
	using direction_id = warthog::grid::direction_id;
	using jump_res_width = std::conditional_t<ChainJump, uint8_t, uint16_t>;
	using jump_res = std::conditional_t<DeadEnd, std::make_signed_t<jump_res_width>, jump_res_width>;
	using length = int32_t;
	struct alignas(int64_t) cell : std::array<jump_res, 8>
	{ };

	std::unique_ptr<cell[]> db;
	uint32_t width = 0;
	uint32_t cells = 0;
	static constexpr uint32_t node_count(uint32_t width, uint32_t height) noexcept
	{
		return static_cast<size_t>(width) * static_cast<size_t>(height);
	}
	constexpr size_t mem() noexcept
	{
		return cells * sizeof(jump_res);
	}

	static consteval length chain_length() noexcept
	{
		constexpr length amax = static_cast<length>(std::numeric_limits<jump_res>::max());
		constexpr length amin = static_cast<length>(std::abs(std::numeric_limits<jump_res>::min()));
		//                   254               127
		return !DeadEnd ? amax - 1 : std::min(amax, amin-1);
	}
	static consteval jump_res chain_value() noexcept
	{
		return DeadEnd ? std::numeric_limits<jump_res>::min() : std::numeric_limits<jump_res>::max();
	}

	/// @brief setup db, init to zero
	void init(uint32_t width, uint32_t height)
	{
		this->cells = node_count(width, height);
		this->width = width;
		this->db = std::make_unique<cell[]>(cells);
	}
	/// @brief sets precomputed jump-point along line [loc...loc+len)
	///        when len reaches 0, final cell is not set
	/// @param d direction of line
	/// @param loc location
	/// @param len length to cover, exclusive end
	void set_line(direction_id d, grid_id loc, length len) noexcept
	{
		// no negative for deadend
		assert(d < 8);
		assert(db != nullptr);
		assert(loc.id < cells);
		assert(DeadEnd || len >= 0);
		assert(std::abs(len) < std::numeric_limits<int16_t>::max());
		int32_t id = static_cast<int32_t>(loc.id);
		const int32_t id_adj = warthog::grid::dir_id_adj(d, width);
		const length len_adj = DeadEnd ? static_cast<length>(len >= 0 ? -1 : 1) : -1;
		while (len != 0) {
			jump_res value;
			if constexpr (!ChainJump) {
				// just use value
				value = static_cast<jump_res>(len);
			} else {
				// if len <= chain_length(), use len, otherwise use chain_value() to force a chain lookup
				if constexpr (DeadEnd) {
					// seperate DeadEnd code to remove abs
					value = std::abs(len) <= chain_length() ? static_cast<jump_res>(len) : chain_value();
				} else {
					value = len <= chain_length() ? static_cast<jump_res>(len) : chain_value();
				}
			}
			db[id][d] = value;
			if constexpr (!ChainJump) break;
			id += id_adj;
			len += len_adj;
		}
	}
	/// @brief get pre-computed jump at location in direction
	/// @param d direction
	/// @param loc location
	/// @return jump from loc in d
	length get_jump(direction_id d, grid_id loc) noexcept
	{
		// no negative for deadend
		assert(d < 8);
		assert(db != nullptr);
		assert(loc.id < cells);
		if constexpr (!ChainJump) {
			return static_cast<length>(db[loc.id][d]);
		} else {
			jump_res u = static_cast<length>(db[loc.id][d]);
			if (u != chain_value()) {
				return static_cast<length>(u);
			} else {
				return chain_jump(d, loc);
			}
		}
	}
	/// @brief get mutable cell
	/// @param loc location
	/// @return cell entry for loc
	cell& get(grid_id loc) noexcept
	{
		assert(db != nullptr);
		assert(loc.id < cells);
		return db[loc.id];
	}
	/// @brief get immutable cell
	/// @param loc location
	/// @return cell entry for loc
	cell get(grid_id loc) const noexcept
	{
		assert(db != nullptr);
		assert(loc.id < cells);
		return db[loc.id];
	}
	/// @brief perform a chain jump, assumes loc is chain value
	///        assumes user manually checked db value at loc and value was chain_value()
	/// @param d direction of jump
	/// @param loc location
	/// @return jump length, at least chain_length()+1
	/// @pre db[loc.id][d] == chain_value(), loc must be chained
	length chain_jump(direction_id d, grid_id loc) noexcept requires(ChainJump)
	{
		assert(db != nullptr);
		assert(loc.id < cells);
		assert(db[loc.id][d] == chain_value());
		int32_t id = loc.id;
		const int32_t id_adj = warthog::grid::dir_id_adj(d, width);
		length len = 0;
		jump_res j;
		do {
			// continue from previous jump
			id += id_adj;
			len += chain_length();
			j = static_cast<length>(db[id][d]);
		} while (j == chain_value());
		assert(j != 0);
		if constexpr (DeadEnd) {
			// check if block and negate
			len = j >= 0 ? (len+j) : -(len-j);
		} else {
			len += j;
		}
		return len;
	}

	/// @brief get jump cell
	/// @param loc location
	/// @return cell
	cell operator[](grid_id loc) const noexcept
	{
		assert(db != nullptr);
		assert(loc.id < cells);
		return db[loc.id];
	}

	std::ostream& print(std::ostream& out)
	{
		std::array<char, 8*8> buffer;
		for (uint32_t i = 0; i < cells; ) {
			for (uint32_t j = 0; j < width; ++j, ++i) {
				// fill buffer with cell
				char *at = buffer.begin(), *end = buffer.end();
				for (int k = 0; k < 8; ++k) {
					jump_distance dist = get_jump(static_cast<direction_id>(k), grid_id{i});
					auto res = std::to_chars(at, end, dist);
					if (res.ec != std::errc{}) {
						// error, exit
						out.setstate(std::ios::failbit);
						return out;
					}
					if (at == end) {
						// out of space, should not happen
						out.setstate(std::ios::failbit);
						return out;
					}
					*at++ = k+1 < 8 ? ',' : '\n';
				}
				out << buffer.data() << (j+1 < width ? '\t' : '\n');
			}
		}
	}
};

template <typename JumpTable = jump_point_table<>, typename OnlinePoint = jump_point_online>
class jump_point_offline : public OnlinePoint
{
public:
	using typename OnlinePoint::bittable;
	using typename OnlinePoint::rotate_grid;
	using jump_point_online::jump_point_online;

	template <direction_id D>
		requires CardinalId<D>
	jump_distance
	jump_cardinal_next(point loc)
	{
		return static_cast<jump_distance>(jump_table_.get_jump(D, this->map_.point_to_id(loc)));
	}
	template <direction_id D>
		requires CardinalId<D>
	jump_distance
	jump_cardinal_next(domain::rgrid_id_t<D> node_id)
	{
		if constexpr (std::same_as<domain::rgrid_id_t<D>, grid_id>) {
			return static_cast<jump_distance>(jump_table_.get_jump(D, node_id));
		} else {
			return static_cast<jump_distance>(jump_table_.get_jump(D, this->map_.rid_to_id(node_id)));
		}
	}
	
	template <direction_id D>
		requires InterCardinalId<D>
	std::pair<uint16_t, jump_distance>
	jump_intercardinal_many(
	    point loc, intercardinal_jump_result* result, uint16_t result_size, jump_distance max_distance = std::numeric_limits<jump_distance>::max())
	{
		constexpr direction_id Dhori = dir_intercardinal_hori(D);
		constexpr direction_id Dvert = dir_intercardinal_vert(D);
		grid_id node_id = this->map_.point_to_id(loc);
		const uint32_t node_adj = dir_id_adj(D, this->map_.width());
		std::pair<uint16_t, jump_distance> res{0,0};
		for (/*res.first*/; res.first < result_size; ) {
			jump_distance dist = static_cast<jump_distance>(jump_table_.get_jump(D, node_id));
			if (dist <= 0) {
				res.second = static_cast<jump_distance>(-res.second + dist);
				break;
			}
			res.first += 1;
			res.second += dist;
			node_id.id += dist * node_adj;
			// found point
			intercardinal_jump_result resi;
			resi.inter = res.second;
			resi.hori = static_cast<jump_distance>(jump_table_.get_jump(Dhori, node_id));
			resi.vert = static_cast<jump_distance>(jump_table_.get_jump(Dvert, node_id));
			*(result++) = resi;
			if (res.second > max_distance)
				break;
		}
		return res;
	}

	void
	set_map(const rotate_grid& map)
	{
		OnlinePoint::set_map(map);
		// compute offline jump-point table
		compute_jump_table();
	}
	void compute_jump_table()
	{
		const uint32_t width = this->map_.width();
		const uint32_t height = this->map_.height();
		const uint32_t rwidth = this->map_.rmap().width();
		jump_table_.init(width, height);
		auto&& point_in_range = [=](point p) noexcept { return p.x < width && p.y < height; };

		// handle cardinal scans

		struct CardinalScan
		{
			direction_id d;
			point start;
			spoint row_adj;
		};
		const std::array<CardinalScan, 4> scans{{
			{NORTH_ID, point(0,height-1), spoint(1,0)},
			{SOUTH_ID, point(0,0), spoint(1,0)},
			{EAST_ID, point(0,0), spoint(0,1)},
			{WEST_ID, point(width-1,0), spoint(0,1)}
		}};

		using jump_cardinal_type = jump_distance(OnlinePoint*, uint32_t);
		warthog::util::for_each_integer_sequence< std::integer_sequence<direction_id, NORTH_ID, EAST_ID, SOUTH_ID, WEST_ID> >(
			[&](auto iv) {
				constexpr direction_id di = decltype(iv)::value;
				const auto map = this->map_[domain::rgrid_index<di>];
				CardinalScan s = *std::find_if(scans.begin(), scans.end(), [di](auto& s) { return s.d == di; });
				// start scan
				point node = s.start;
				const spoint adj = dir_unit_point(s.d);
				for (point node = s.start; point_in_range(node); node = node + adj) {
					point current_node = node;
					while (point_in_range(current_node)) {
						auto current_id = this->map_.template point_to_id_d<di>(current_node);
						if (map.get(static_cast<grid_id>(current_id))) {
							jump_distance d = OnlinePoint::template jump_cardinal_next<di>(current_id);
							if (d == 0)[[unlikely]] {
								// immediently blocked
								// jump_table_.get(row_node)[di] = 0;
								current_node = current_node + adj; // we know the next cell is a blocker
								// assert(!this->map_.map().get(grid_id{node.id + col_adj}));
							} else {
								// store result
								jump_table_.set_line(di, this->map_.point_to_id(current_node), d);
								current_node = current_node + std::abs(d) * adj; // next cell is the reached cell
								assert(point_in_range(current_node)); // j should never jump past edge
								// assert(this->map_.map().get(grid_id{node.id + j * col_adj}));
							}
						} else {
							current_node = current_node + adj; // is invalid cell, check the next
						}
					}
				}
			}
		);

		//
		// InterCardinal scans
		//
		struct ICardinalScan
		{
			direction_id d;
			point start; // start location
		};
		const std::array<ICardinalScan, 4> Iscans{{
			{NORTHEAST_ID, point(0,height-1)},
			{NORTHWEST_ID, point(width-1,height-1)},
			{SOUTHEAST_ID, point(0,0)},
			{SOUTHWEST_ID, point(width-1,0)}
		}};

		for (auto s : Iscans) {
			const direction_id dh = dir_intercardinal_hori(s.d);
			const direction_id dv = dir_intercardinal_hori(s.d);
			const spoint adj = dir_unit_point(s.d);
			for (int axis = 0; axis < 2; ++axis) {
				// 0 = follow hori, then follow vert
				point start = s.start;
				while (true) { // start location
					if (axis == 0) start.x += static_cast<uint32_t>(adj.x);
					else start.y += static_cast<uint32_t>(adj.y);
					if (!point_in_range(start))
						break; // out of bounds, do nothing
					// now scan along the diagonal
					point loc = start;
					while (true) {
						if (!point_in_range(loc))
							break; // out of bounds, end
						if (this->map_.map().get(this->map_.point_to_id(loc))) {
							// block cell
							// this->jump_table_.get(this->map_.point_to_id(loc))[s.d] = 0;
							loc = loc + adj;
							continue;
						}
						// calc distance
						point currentloc = loc;
						jump_distance dist = 0;
						while (true) {
							point nextloc = currentloc + adj;
							if (point_in_range(nextloc) && this->map_.map().get(this->map_.point_to_id(nextloc))) {
								// traversable tile
								dist += 1;
								auto cell = jump_table_[this->map_.point_to_id(nextloc)];
								if (cell[dh] > 0 || cell[dv] > 0) {
									// end point here
									jump_table_.set_line(s.d, this->map_.point_to_id(currentloc), dist);
									loc = nextloc;
									break; // done with this line
								}
							} else {
								// reached blocker or edge of map
								if (dist != 0) {
									jump_table_.set_line(s.d, this->map_.point_to_id(currentloc), -dist);
								}
								loc = nextloc + adj; // skip nextloc as this is a blocker
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
