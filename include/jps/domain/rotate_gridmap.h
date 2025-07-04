#ifndef WARTHOG_DOMAIN_ROTATE_GRIDMAP_H
#define WARTHOG_DOMAIN_ROTATE_GRIDMAP_H

// domain::gridmap.h
//
// A uniform cost domain::gridmap implementation.  The map is stored in
// a compact matrix form. Nodes are represented as single bit quantities.
//

#include <jps/forward.h>
#include <warthog/domain/grid.h>
#include <warthog/domain/gridmap.h>
#include <array>
#include <warthog/constants.h>
#include <utility>
#include <memory>

namespace jps::domain
{

namespace details {

template <auto T>
struct direction_grid_id;
template <>
struct direction_grid_id<NORTH_ID>
{
	using type = rgrid_id;
	static constexpr int map_id = 1;
	static constexpr bool east = true;
};
template <>
struct direction_grid_id<EAST_ID>
{
	using type = grid_id;
	static constexpr int map_id = 0;
	static constexpr bool east = true;
};
template <>
struct direction_grid_id<SOUTH_ID>
{
	using type = rgrid_id;
	static constexpr int map_id = 1;
	static constexpr bool east = false;
};
template <>
struct direction_grid_id<WEST_ID>
{
	using type = grid_id;
	static constexpr int map_id = 0;
	static constexpr bool east = false;
};
template <>
struct direction_grid_id<NORTH>
{
	using type = rgrid_id;
	static constexpr int map_id = 1;
};
template <>
struct direction_grid_id<EAST>
{
	using type = grid_id;
	static constexpr int map_id = 0;
};
template <>
struct direction_grid_id<SOUTH>
{
	using type = rgrid_id;
	static constexpr int map_id = 1;
};
template <>
struct direction_grid_id<WEST>
{
	using type = grid_id;
	static constexpr int map_id = 0;
};

} // namespace details

/// @brief returns id type for cardinal direction, {EAST,EAST_ID,WEST,WEST_ID} = grid_id; {NORTH,NORTH_ID,SOUTH,SOUTH_ID} = rgrid_id;
/// @tparam D value in direction or direction_id
template <auto D>
using rgrid_id_t = typename details::direction_grid_id<D>::type;

template <auto D>
constexpr inline int rgrid_index = details::direction_grid_id<D>::map_id;
template <auto D>
constexpr inline bool rgrid_east = details::direction_grid_id<D>::east;

using ::warthog::domain::gridmap;

struct rgridmap_point_conversions
{
	uint16_t map_unpad_height_m1_ = 0;
	uint16_t map_pad_width_ = 0;
	uint16_t rmap_pad_width_ = 0;

	void conv_assign(const gridmap& map, const gridmap& rmap) noexcept
	{
		map_unpad_height_m1_ = static_cast<uint16_t>(map.header_height() - 1);
		map_pad_width_ = static_cast<uint16_t>(map.width());
		rmap_pad_width_ = static_cast<uint16_t>(rmap.width());
	}

	point
	point_to_rpoint(point p) const noexcept
	{
		return {
			static_cast<uint16_t>(map_unpad_height_m1_ - p.y),
			static_cast<uint16_t>(p.x)};
	}
	point
	rpoint_to_point(point p) const noexcept
	{
		return {
			static_cast<uint16_t>(p.y),
			static_cast<uint16_t>(map_unpad_height_m1_ - p.x)};
	}
	grid_id
	point_to_id(point p) const noexcept
	{
		return grid_id{
			static_cast<grid_id::id_type>(p.y + domain::gridmap::PADDED_ROWS)
				* map_pad_width_
			+ static_cast<grid_id::id_type>(p.x)};
	}
	rgrid_id
	rpoint_to_rid(point p) const noexcept
	{
		return rgrid_id{
			static_cast<rgrid_id::id_type>(p.y + domain::gridmap::PADDED_ROWS)
				* rmap_pad_width_
			+ static_cast<rgrid_id::id_type>(p.x)};
	}
	point
	id_to_point(grid_id p) const noexcept
	{
		return {
			static_cast<uint16_t>(p.id % map_pad_width_),
			static_cast<uint16_t>(p.id / map_pad_width_ - domain::gridmap::PADDED_ROWS)};
	}
	point
	rid_to_rpoint(rgrid_id p) const noexcept
	{
		return {
			static_cast<uint16_t>(p.id % rmap_pad_width_),
			static_cast<uint16_t>(
				p.id / rmap_pad_width_ - domain::gridmap::PADDED_ROWS)};
	}
	rgrid_id
	id_to_rid(grid_id mapid) const noexcept
	{
		assert(!mapid.is_none());
		return rpoint_to_rid(point_to_rpoint(id_to_point(mapid)));
	}
	grid_id
	rid_to_id(rgrid_id mapid) const noexcept
	{
		assert(!mapid.is_none());
		return point_to_id(rpoint_to_point(rid_to_rpoint(mapid)));
	}

	template <auto D, ::warthog::Identity GridId>
		requires std::same_as<GridId, grid_id> || std::same_as<GridId, rgrid_id>
	rgrid_id_t<D> to_id_d(GridId id) const noexcept
	{
		using res_type = rgrid_id_t<D>;
		if constexpr (std::same_as<GridId, res_type>) {
			return id; // is same as output, do nothing
		} else if constexpr (std::same_as<res_type, grid_id>) {
			return rid_to_id(id);
		} else {
			return id_to_rid(id);
		}
	}

	template <auto D>
	rgrid_id_t<D> point_to_id_d(point loc) const noexcept
	{
		using res_type = rgrid_id_t<D>;
		if constexpr (std::same_as<res_type, grid_id>) {
			return point_to_id(loc);
		} else {
			return rpoint_to_rid(point_to_rpoint(loc));
		}
	}

	template <auto D>
	rgrid_id_t<D> rpoint_to_id_d(point loc) const noexcept
	{
		using res_type = rgrid_id_t<D>;
		if constexpr (std::same_as<res_type, grid_id>) {
			return point_to_id(rpoint_to_point(loc));
		} else {
			return rpoint_to_rid(loc);
		}
	}
};

struct gridmap_rotate_ptr : std::array<domain::gridmap*, 2>
{
	gridmap_rotate_ptr() : array{}
	{ }
	gridmap_rotate_ptr(domain::gridmap& l_map, domain::gridmap& l_rmap) noexcept
		: array{&l_map, &l_rmap}
	{ }
	domain::gridmap& map() noexcept { return *(*this)[0]; }
	const domain::gridmap& map() const noexcept { return *(*this)[0]; }
	domain::gridmap& rmap() noexcept { return *(*this)[1]; }
	const domain::gridmap& rmap() const noexcept { return *(*this)[1]; }
	operator bool() const noexcept { return (*this)[0]; }
};
struct gridmap_rotate_ptr_convs : gridmap_rotate_ptr, rgridmap_point_conversions
{
	gridmap_rotate_ptr_convs() = default;
	gridmap_rotate_ptr_convs(domain::gridmap& l_map, domain::gridmap& l_rmap) noexcept
		: gridmap_rotate_ptr(l_map, l_rmap), rgridmap_point_conversions{static_cast<uint16_t>(l_map.header_height()-1), static_cast<uint16_t>(l_map.width()), static_cast<uint16_t>(l_rmap.width())}
	{ }
	gridmap_rotate_ptr_convs(gridmap_rotate_ptr maps) noexcept
		: gridmap_rotate_ptr(maps), rgridmap_point_conversions{}
	{
		if (*this) {
			conv_assign(map(), rmap());
		}
	}
};
struct gridmap_rotate_table : std::array<domain::gridmap::bittable, 2>
{
	gridmap_rotate_table() : array{}
	{ }
	gridmap_rotate_table(domain::gridmap& l_map, domain::gridmap& l_rmap) noexcept
		: array{l_map, l_rmap}
	{ }
	gridmap_rotate_table(domain::gridmap::bittable& l_map, domain::gridmap::bittable& l_rmap) noexcept
		: array{l_map, l_rmap}
	{ }
	domain::gridmap::bittable& map() noexcept { return (*this)[0]; }
	const domain::gridmap::bittable& map() const noexcept { return (*this)[0]; }
	domain::gridmap::bittable& rmap() noexcept { return (*this)[1]; }
	const domain::gridmap::bittable& rmap() const noexcept { return (*this)[1]; }
	operator bool() const noexcept { return (*this)[0].data(); }
};
struct gridmap_rotate_table_convs : gridmap_rotate_table, rgridmap_point_conversions
{
	gridmap_rotate_table_convs() = default;
	gridmap_rotate_table_convs(domain::gridmap& l_map, domain::gridmap& l_rmap) noexcept
		: gridmap_rotate_table(l_map, l_rmap), rgridmap_point_conversions(l_map.header_height() - 1)
	{ }
	gridmap_rotate_table_convs(gridmap_rotate_table maps, uint16_t map_unpad_height_m1) noexcept
		: gridmap_rotate_table(maps), rgridmap_point_conversions{map_unpad_height_m1, static_cast<uint16_t>(maps[0].width()), static_cast<uint16_t>(maps[1].width())}
	{ }
};

class rotate_gridmap : public rgridmap_point_conversions
{
private:
	std::unique_ptr<domain::gridmap> rmap_obj;
	gridmap_rotate_ptr maps = {};

public:
	rotate_gridmap() = default;
	rotate_gridmap(domain::gridmap& map, domain::gridmap* rmap = nullptr)
	{
		if (rmap != nullptr) {
			maps[0] = &map;
			maps[1] = rmap;
			conv_assign(map, *rmap);
		} else {
			create_rmap(map);
		}
	}

	void link(gridmap_rotate_ptr rmap)
	{
		rmap_obj = nullptr;
		if (rmap) {
			maps = rmap;
			conv_assign(*maps[0], *maps[1]);
		} else {
			clear();
		}
	}
	void clear()
	{
		rmap_obj = nullptr;
		maps = {};
		static_cast<rgridmap_point_conversions&>(*this) = {};
	}
	void create_rmap(domain::gridmap& map)
	{
		maps[0] = &map;
		
		const uint32_t maph = map.header_height();
		const uint32_t mapw = map.header_width();
		auto tmap           = std::make_unique<domain::gridmap>(mapw, maph);

		for(uint32_t y = 0; y < maph; y++)
		{
			for(uint32_t x = 0; x < mapw; x++)
			{
				bool label = map.get_label(map.to_padded_id_from_unpadded(x, y));
				uint32_t rx = (maph - 1) - y;
				uint32_t ry = x;
				tmap->set_label(tmap->to_padded_id_from_unpadded(rx, ry), label);
			}
		}

		// set values
		rmap_obj = std::move(tmap);
		maps[1] = rmap_obj.get();
		conv_assign(*maps[0], *maps[1]);
	}

	domain::gridmap& map() noexcept
	{
		assert(maps[0] != nullptr);
		return *maps[0];
	}
	const domain::gridmap& map() const noexcept
	{
		assert(maps[0] != nullptr);
		return *maps[0];
	}
	domain::gridmap& rmap() noexcept
	{
		assert(maps[1] != nullptr);
		return *maps[1];
	}
	const domain::gridmap& rmap() const noexcept
	{
		assert(maps[1] != nullptr);
		return *maps[1];
	}

	operator bool() const noexcept
	{
		return maps[0] != nullptr;
	}
	operator gridmap_rotate_ptr() const noexcept
	{
		assert(maps[0] != nullptr && maps[1] != nullptr);
		return maps;
	}
	operator gridmap_rotate_ptr_convs() const noexcept
	{
		assert(maps[0] != nullptr && maps[1] != nullptr);
		return gridmap_rotate_ptr_convs(maps);
	}
	operator gridmap_rotate_table() const noexcept
	{
		assert(maps[0] != nullptr && maps[1] != nullptr);
		return gridmap_rotate_table(*maps[0], *maps[1]);
	}
	operator gridmap_rotate_table_convs() const noexcept
	{
		assert(maps[0] != nullptr && maps[1] != nullptr);
		return gridmap_rotate_table_convs(*this, map_unpad_height_m1_);
	}
};

} // namespace warthog::grid

#endif // WARTHOG_DOMAIN_GRIDMAP_H
