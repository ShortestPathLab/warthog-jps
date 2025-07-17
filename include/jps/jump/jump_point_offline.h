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
#ifndef NDEBUG
	uint32_t d_cells = 0;
#endif
	static constexpr uint32_t node_count(uint32_t width, uint32_t height) noexcept
	{
		return static_cast<size_t>(width) * static_cast<size_t>(height);
	}
	static constexpr size_t mem(uint32_t width, uint32_t height) noexcept
	{
		return node_count(width, height) * sizeof(jump_res);
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
		db = std::make_unique<jump_res[]>(node_count(width, height));
		this->width = width;
#ifndef NDEBUG
		this->d_cells = node_count(width, height);
#endif
	}
	/// @brief sets precomputed jump-point along line [loc...loc+len)
	///        when len reaches 0, final cell is not set
	/// @param d direction of line
	/// @param loc location
	/// @param len length to cover, exclusive end
	void set_line(direction_id d, jps_id loc, length len) noexcept
	{
		// no negative for deadend
		assert(d < 8);
		assert(db != nullptr);
		assert(loc.id < d_cells);
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
			id += id_adj;
			len += len_adj;
		}
	}
	/// @brief get pre-computed jump at location in direction
	/// @param d direction
	/// @param loc location
	/// @return jump from loc in d
	length get_jump(direction_id d, jps_id loc) noexcept
	{
		// no negative for deadend
		assert(d < 8);
		assert(db != nullptr);
		assert(loc.id < d_cells);
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
	/// @brief perform a chain jump, assumes loc is chain value
	///        assumes user manually checked db value at loc and value was chain_value()
	/// @param d direction of jump
	/// @param loc location
	/// @return jump length, at least chain_length()+1
	/// @pre db[loc.id][d] == chain_value(), loc must be chained
	length chain_jump(direction_id d, jps_id loc) noexcept requires(ChainJump)
	{
		assert(db != nullptr);
		assert(loc.id < d_cells);
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
	cell operator[](jps_id loc) const noexcept
	{
		assert(db != nullptr);
		assert(loc.id < d_cells);
		return db[loc.id];
	}
};

template <typename JumpTable = jump_point_table<>, typename OnlinePoint = jump_point_online>
class jump_point_offline : public OnlinePoint
{
public:
	using jump_point_online::jump_point_online;

	template <direction_id D>
		requires CardinalId<D>
	jump_distance
	jump_cardinal_next(point loc)
	{
		if constexpr (domain::rgrid_index<D> == 0) {
			return jump_cardinal_next(this->map_.point_to_id(loc));
		} else {
			return jump_cardinal_next(this->map_.rpoint_to_rid(this->map_.point_to_rpoint(loc)));
		}
	}
	template <direction_id D>
		requires CardinalId<D>
	jump_distance
	jump_cardinal_next(domain::rgrid_id_t<D> node_id)
	{

	}

protected:
	JumpTable jump_table_;
};

} // namespace jps::jump

#endif // JPS_JUMP_JUMP_POINT_OFFLINE_H
