#ifndef WARTHOG_DOMAIN_ROTATE_GRIDMAP_H
#define WARTHOG_DOMAIN_ROTATE_GRIDMAP_H

//
// jps/domain/rotate_gridmap.h
//
// An extended domain of gridmap that uses 2 gridmaps, one rotated
// for use in JPS block-based jump-point location.
//
// Class rotate_gridmap is the main implementation.
// It supports pointing to a user-supplied gridmap and rotated gridmap,
// or just a user provided gridmap, which it would generate and own the
// rotated gridmap.
//
// Small copyable storage of these maps are provided in utility structs:
// gridmap_rotate_ptr, gridmap_rotate_ptr_convs: 2 pointers to gridmap,
//   accessing the grid would require 2-level of indirects (gridmap->data[i])
// gridmap_rotate_table, gridmap_rotate_table_convs: 2 gridmap::bitarray,
//   accessing the grid is only a single redirect
// Use of these datatypes do not copy the tables themselves, and are quite
// compact. The non _convs variants should be 2-pointer sized (16-bytes). The
// _convs variants are supplied with width and height, allowing for x/y and
// grid_id and rgrid_id conversions between the two grid (24-bytes).
//
// @author Ryan Hechenberger
// @created 2025-11-20
//

#include <array>
#include <jps/forward.h>
#include <memory>
#include <utility>
#include <warthog/constants.h>
#include <warthog/domain/grid.h>
#include <warthog/domain/gridmap.h>

namespace jps::domain
{

namespace details
{

template<auto T>
struct direction_grid_id;
template<>
struct direction_grid_id<NORTH_ID>
{
	using type                     = rgrid_id;
	static constexpr size_t map_id = 1;
	static constexpr bool east     = true;
};
template<>
struct direction_grid_id<EAST_ID>
{
	using type                     = grid_id;
	static constexpr size_t map_id = 0;
	static constexpr bool east     = true;
};
template<>
struct direction_grid_id<SOUTH_ID>
{
	using type                     = rgrid_id;
	static constexpr size_t map_id = 1;
	static constexpr bool east     = false;
};
template<>
struct direction_grid_id<WEST_ID>
{
	using type                     = grid_id;
	static constexpr size_t map_id = 0;
	static constexpr bool east     = false;
};
template<>
struct direction_grid_id<NORTH>
{
	using type                     = rgrid_id;
	static constexpr size_t map_id = 1;
	static constexpr bool east     = true;
};
template<>
struct direction_grid_id<EAST>
{
	using type                     = grid_id;
	static constexpr size_t map_id = 0;
	static constexpr bool east     = true;
};
template<>
struct direction_grid_id<SOUTH>
{
	using type                     = rgrid_id;
	static constexpr size_t map_id = 1;
	static constexpr bool east     = false;
};
template<>
struct direction_grid_id<WEST>
{
	using type                     = grid_id;
	static constexpr size_t map_id = 0;
	static constexpr bool east     = false;
};

template<size_t I>
struct index_grid_id;
template<>
struct index_grid_id<0>
{
	using type = grid_id;
};
template<>
struct index_grid_id<1>
{
	using type = rgrid_id;
};

template<warthog::Identity Grid>
struct grid_identity;
template<>
struct grid_identity<grid_id>
{
	using type                    = grid_id;
	static constexpr size_t index = 0;
};
template<>
struct grid_identity<rgrid_id>
{
	using type                    = rgrid_id;
	static constexpr size_t index = 1;
};

} // namespace details

/// @brief returns id type for cardinal direction, {EAST,EAST_ID,WEST,WEST_ID}
/// = grid_id; {NORTH,NORTH_ID,SOUTH,SOUTH_ID} = rgrid_id;
/// @tparam D value in direction or direction_id
template<auto D>
using grid_id_dir_t = typename details::direction_grid_id<D>::type;
/// @brief returns id type of map index I
template<size_t I>
using grid_id_index_t = typename details::index_grid_id<I>::type;
/// @brief returns id type for cardinal direction, {EAST,EAST_ID,WEST,WEST_ID}
/// = grid_id; {NORTH,NORTH_ID,SOUTH,SOUTH_ID} = rgrid_id;
/// @tparam D value in direction or direction_id
template<warthog::Identity D>
using grid_identity = details::grid_identity<std::remove_cvref_t<D>>;

template<auto D>
constexpr inline size_t rgrid_index = details::direction_grid_id<D>::map_id;
template<auto D>
constexpr inline bool rgrid_east = details::direction_grid_id<D>::east;
template<auto D>
constexpr inline bool rgrid_hori = rgrid_index<D> == 0;

using ::warthog::domain::gridmap;

struct grid_pair_id
{
	grid_id g;  ///< grid_id for horizontal id
	rgrid_id r; ///< grid_id for vertical id
};
/// @return id.g for I==0, id.r for I==1
template<size_t I>
constexpr auto
get(const grid_pair_id& id) noexcept
{
	static_assert(I < 2, "0 = grid_id, 1 = rgird_id");
	if constexpr(I == 0) { return id.g; }
	else { return id.r; }
}
/// @return id.g for I==0, id.r for I==1
template<size_t I>
constexpr auto&
get(grid_pair_id& id) noexcept
{
	static_assert(I < 2, "0 = grid_id, 1 = rgird_id");
	if constexpr(I == 0) { return id.g; }
	else { return id.r; }
}
/// @return id.g for grid_id, id.r for rgrid_id
template<warthog::Identity T>
constexpr T
get(const grid_pair_id& id) noexcept
{
	return get<grid_identity<T>::index>(id);
}
/// @return id.g for grid_id, id.r for rgrid_id
template<warthog::Identity T>
constexpr T&
get(grid_pair_id& id) noexcept
{
	return get<grid_identity<T>::index>(id);
}
/// @return get<grid_id|rgrid_id>(id) with associated horizontal/vertical of D
template<direction_id D>
constexpr grid_id_dir_t<D>
get_d(const grid_pair_id& id) noexcept
{
	return get<rgrid_index<D>>(id);
}
/// @return get<grid_id|rgrid_id>(id) with associated horizontal/vertical of D
template<direction_id D>
constexpr grid_id_dir_t<D>&
get_d(grid_pair_id& id) noexcept
{
	return get<rgrid_index<D>>(id);
}
/// @return get<grid_id|rgrid_id>(id) with associated horizontal/vertical of D
template<direction D>
constexpr grid_id_dir_t<D>
get_d(const grid_pair_id& id) noexcept
{
	return get<rgrid_index<D>>(id);
}
/// @return get<grid_id|rgrid_id>(id) with associated horizontal/vertical of D
template<direction D>
constexpr grid_id_dir_t<D>&
get_d(grid_pair_id& id) noexcept
{
	return get<rgrid_index<D>>(id);
}

/// @brief class with functions to handle conversions from/to grid and rgrid,
/// designed to be inherited by other classes for functionality.
struct rgridmap_point_conversions
{
	static constexpr uint16_t XADJ
	    = uint16_t(-1u); // width adjustment to rotate for height
	static constexpr uint16_t YADJ
	    = gridmap::PADDED_ROWS;               // height adjustment to rotate
	std::array<uint16_t, 2> map_height_ = {}; // height + XADJ
	std::array<uint16_t, 2> map_width_  = {};
	// uint16_t map_width_ = 0;
	// uint16_t map_height_m1p_ = 0;
	// uint16_t rmap_width_ = 0;
	// uint16_t rmap_height_ = 0;

	void
	conv_assign(gridmap::bittable map, gridmap::bittable rmap) noexcept
	{
		map_height_
		    = {static_cast<uint16_t>(map.height() + XADJ),
		       static_cast<uint16_t>(rmap.height() + XADJ)};
		map_width_
		    = {static_cast<uint16_t>(map.width()),
		       static_cast<uint16_t>(rmap.width())};
	}

	/// @return returns the padded width of map
	uint16_t
	width() const noexcept
	{
		return map_width_[0];
	}
	/// @return returns the padded height of map
	uint16_t
	height() const noexcept
	{
		return uint16_t(map_height_[0] - XADJ);
	}

	/// @return returns the padded width of map
	uint16_t
	rwidth() const noexcept
	{
		return map_width_[1];
	}
	/// @return returns the padded height of map
	uint16_t
	rheight() const noexcept
	{
		return uint16_t(map_height_[1] - XADJ);
	}

	point
	point_to_rpoint(point p) const noexcept
	{
		return {
		    static_cast<uint16_t>(map_height_[0] - p.y),
		    static_cast<uint16_t>(p.x + YADJ)};
	}
	point
	rpoint_to_point(point p) const noexcept
	{
		return {
		    static_cast<uint16_t>(p.y - YADJ),
		    static_cast<uint16_t>(map_height_[0] - p.x)};
	}
	grid_id
	point_to_id(point p) const noexcept
	{
		return grid_id{
		    static_cast<grid_id::id_type>(p.y) * map_width_[0]
		    + static_cast<grid_id::id_type>(p.x)};
	}
	rgrid_id
	rpoint_to_rid(point p) const noexcept
	{
		return rgrid_id{
		    static_cast<rgrid_id::id_type>(p.y) * map_width_[1]
		    + static_cast<rgrid_id::id_type>(p.x)};
	}
	point
	id_to_point(grid_id p) const noexcept
	{
		return {
		    static_cast<uint16_t>(p.id % map_width_[0]),
		    static_cast<uint16_t>(p.id / map_width_[0])};
	}
	point
	rid_to_rpoint(rgrid_id p) const noexcept
	{
		return {
		    static_cast<uint16_t>(p.id % map_width_[1]),
		    static_cast<uint16_t>(p.id / map_width_[1])};
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
	grid_pair_id
	point_to_pair_id(point loc) const noexcept
	{
		return grid_pair_id{
		    point_to_id(loc), rpoint_to_rid(point_to_rpoint(loc))};
	}

	template<auto D, ::warthog::Identity GridId>
	    requires std::same_as<GridId, grid_id>
	    || std::same_as<GridId, rgrid_id>
	grid_id_dir_t<D>
	to_id_d(GridId id) const noexcept
	{
		using res_type = grid_id_dir_t<D>;
		if constexpr(std::same_as<GridId, res_type>)
		{
			return id; // is same as output, do nothing
		}
		else if constexpr(std::same_as<res_type, grid_id>)
		{
			return rid_to_id(id);
		}
		else { return id_to_rid(id); }
	}

	template<auto D>
	grid_id_dir_t<D>
	point_to_id_d(point loc) const noexcept
	{
		using res_type = grid_id_dir_t<D>;
		if constexpr(std::same_as<res_type, grid_id>)
		{
			return point_to_id(loc);
		}
		else { return rpoint_to_rid(point_to_rpoint(loc)); }
	}

	template<auto D>
	grid_id_dir_t<D>
	rpoint_to_id_d(point loc) const noexcept
	{
		using res_type = grid_id_dir_t<D>;
		if constexpr(std::same_as<res_type, grid_id>)
		{
			return point_to_id(rpoint_to_point(loc));
		}
		else { return rpoint_to_rid(loc); }
	}
};

/// @brief a copy-by-value class pointing to grid/rgrid
struct gridmap_rotate_ptr : std::array<domain::gridmap*, 2>
{
	gridmap_rotate_ptr() : array{} { }
	gridmap_rotate_ptr(
	    domain::gridmap& l_map, domain::gridmap& l_rmap) noexcept
	    : array{&l_map, &l_rmap}
	{ }
	domain::gridmap&
	map() noexcept
	{
		return *(*this)[0];
	}
	const domain::gridmap&
	map() const noexcept
	{
		return *(*this)[0];
	}
	domain::gridmap&
	rmap() noexcept
	{
		return *(*this)[1];
	}
	const domain::gridmap&
	rmap() const noexcept
	{
		return *(*this)[1];
	}
	operator bool() const noexcept { return (*this)[0]; }
};
/// @brief a copy-by-value class pointing to grid/rgrid with id conversions
/// functions
struct gridmap_rotate_ptr_convs : gridmap_rotate_ptr,
                                  rgridmap_point_conversions
{
	gridmap_rotate_ptr_convs() = default;
	gridmap_rotate_ptr_convs(
	    domain::gridmap& l_map, domain::gridmap& l_rmap) noexcept
	    : gridmap_rotate_ptr(l_map, l_rmap)
	{
		conv_assign(l_map, l_rmap);
	}
	gridmap_rotate_ptr_convs(gridmap_rotate_ptr maps) noexcept
	    : gridmap_rotate_ptr(maps)
	{
		if(*this) { conv_assign(map(), rmap()); }
	}
};
/// @brief a copy-by-value class for fast access to grid/rgrid
///        differs from gridmap_rotate_ptr with direct pointer to bit array,
///        instead of pointer to gridmap to bit array
struct gridmap_rotate_table : std::array<domain::gridmap::bitarray, 2>
{
	gridmap_rotate_table() : array{} { }
	gridmap_rotate_table(
	    domain::gridmap& l_map, domain::gridmap& l_rmap) noexcept
	    : array{l_map, l_rmap}
	{ }
	gridmap_rotate_table(
	    domain::gridmap::bittable& l_map,
	    domain::gridmap::bittable& l_rmap) noexcept
	    : array{l_map, l_rmap}
	{ }
	domain::gridmap::bitarray
	map() const noexcept
	{
		return (*this)[0];
	}
	domain::gridmap::bitarray
	rmap() const noexcept
	{
		return (*this)[1];
	}
	operator bool() const noexcept { return (*this)[0].data(); }
};
/// @brief a copy-by-value class for fast access to grid/rgrid with id
/// conversions functions
struct gridmap_rotate_table_convs : gridmap_rotate_table,
                                    rgridmap_point_conversions
{
	gridmap_rotate_table_convs() = default;
	gridmap_rotate_table_convs(
	    domain::gridmap& l_map, domain::gridmap& l_rmap) noexcept
	    : gridmap_rotate_table(l_map, l_rmap)
	{
		conv_assign(l_map, l_rmap);
	}
	gridmap_rotate_table_convs(
	    gridmap_rotate_table maps, rgridmap_point_conversions conv) noexcept
	    : gridmap_rotate_table(maps), rgridmap_point_conversions{conv}
	{ }
	domain::gridmap::bittable
	table() noexcept
	{
		return domain::gridmap::bittable((*this)[0], width(), height());
	}
	domain::gridmap::bittable
	rtable() noexcept
	{
		return domain::gridmap::bittable((*this)[1], rwidth(), rheight());
	}
	domain::gridmap::bittable
	table(size_t i) noexcept
	{
		return domain::gridmap::bittable(
		    (*this)[i], map_width_[i], uint16_t(map_height_[i] - XADJ));
	}
};

/// The main rotate gridmap class.
/// Stores a pointer to a gridmap and rotated gridmap.
/// The rotated gridmap memory may be created and owned by this class, or
/// provided and controlled by the user.
///
/// Supports static_cast operations for all gridmap_rotate_(ptr|table)(_conv)?.
/// They will act as valid small pointer classes (16-24 bytes) with various
/// levels if indirection to the grid data.
class rotate_gridmap : public rgridmap_point_conversions
{
private:
	std::unique_ptr<domain::gridmap> rmap_obj;
	gridmap_rotate_ptr maps = {};

public:
	rotate_gridmap() = default;
	rotate_gridmap(domain::gridmap& map, domain::gridmap* rmap = nullptr)
	{
		if(rmap != nullptr)
		{
			maps[0] = &map;
			maps[1] = rmap;
			conv_assign(map, *rmap);
		}
		else { create_rmap(map); }
	}

	void
	link(gridmap_rotate_ptr rmap)
	{
		rmap_obj = nullptr;
		if(rmap)
		{
			maps = rmap;
			conv_assign(*maps[0], *maps[1]);
		}
		else { clear(); }
	}
	void
	clear()
	{
		rmap_obj                                        = nullptr;
		maps                                            = {};
		static_cast<rgridmap_point_conversions&>(*this) = {};
	}
	void
	create_rmap(domain::gridmap& map)
	{
		maps[0] = &map;

		const uint32_t maph    = map.height();
		const uint32_t maphmp1 = static_cast<uint16_t>(
		    maph + XADJ); // cast required to handle signed value in uint16_t
		const uint32_t mapw = map.width();
		auto tmap           = std::make_unique<domain::gridmap>(mapw, maph);

		for(uint32_t y = 0; y < maph; y++)
		{
			for(uint32_t x = 0; x < mapw; x++)
			{
				bool label = map.get_label(map.to_padded_id_from_padded(x, y));
				uint32_t rx = maphmp1 - y;
				uint32_t ry = static_cast<uint16_t>(x + YADJ);
				tmap->set_label(tmap->to_padded_id_from_padded(rx, ry), label);
			}
		}

		// set values
		rmap_obj = std::move(tmap);
		maps[1]  = rmap_obj.get();
		conv_assign(*maps[0], *maps[1]);

#ifndef NDEBUG
		for(uint32_t y = 0; y < maph; y++)
		{
			for(uint32_t x = 0; x < mapw; x++)
			{
				auto p = this->point_to_rpoint(point(x, y));
				assert(
				    maps[0]->get_label(this->point_to_id(point(x, y)))
				    == maps[1]->get_label(
				        static_cast<grid_id>(this->rpoint_to_rid(
				            this->point_to_rpoint(point(x, y))))));
			}
		}
#endif // NDEBUG
	}

	domain::gridmap&
	map() noexcept
	{
		assert(maps[0] != nullptr);
		return *maps[0];
	}
	const domain::gridmap&
	map() const noexcept
	{
		assert(maps[0] != nullptr);
		return *maps[0];
	}
	domain::gridmap&
	rmap() noexcept
	{
		assert(maps[1] != nullptr);
		return *maps[1];
	}
	const domain::gridmap&
	rmap() const noexcept
	{
		assert(maps[1] != nullptr);
		return *maps[1];
	}

	operator bool() const noexcept { return maps[0] != nullptr; }
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
		return gridmap_rotate_table_convs(
		    *this, static_cast<rgridmap_point_conversions>(*this));
	}

	size_t
	mem() const noexcept
	{
		return rmap_obj != nullptr ? rmap_obj->mem() : 0;
	}
};

} // namespace warthog::grid

#endif // WARTHOG_DOMAIN_GRIDMAP_H
