#ifndef JPS_SEARCH_JPS_CACHE_PRUNE_EXPANSION_POLICY_H
#define JPS_SEARCH_JPS_CACHE_PRUNE_EXPANSION_POLICY_H

///
/// jps_cache_expansion_policy.h
///
/// @author Ryan Hechenberger
/// @date 2026-09-10
///

#include "jps_expansion_policy_base.h"

#include <jps/jump/jump.h>
#include <jps/jump/jump_point_online.h>
#include <jps/domain/rotate_gridmap.h>
#include <warthog/search/gridmap_expansion_policy.h>
#include <warthog/util/template.h>

#include <warthog/memory/alloc/bump_factory.h>
#include <warthog/memory/alloc/indexed_block_factory.h>
#include <warthog/memory/alloc/factory_pointer.h>
#include <warthog/memory/alloc/single_factory.h>

#include <limits>
#include <stdexcept>
#include <cstddef>
#include <vector>

namespace jps::search
{

struct jps_cache_node
{
	using size_type = uint16_t;
	alignas(uint64_t) std::array<uint8_t, 8> success_dir;
	alignas(uint64_t) std::array<jump::jump_distance, 4> cardinal_dist;
	alignas(uint64_t) std::array<size_type, 4> intercardinal_len;
	std::array<jump::intercardinal_jump_result[], 4> intercardinal;

	bool is_cardinal_set(int i) noexcept
	{
		assert(0 <= i && i < 4);
		return cardinal_dist[i] == std::numeric_limits<jump::jump_distance>::min();
	}
	bool is_intercardinal_set(int i) noexcept
	{
		assert(0 <= i && i < 4);
		return intercardinal_len[i] == std::numeric_limits<size_type>::max();
	}

	void init() noexcept
	{
		std::fill_n(success_dir.data(), 8, 0xFFu);
		std::fill_n(cardinal_dist.data(), 4, std::numeric_limits<jump::jump_distance>::min());
		std::fill_n(intercardinal_len.data(), 4, std::numeric_limits<size_type>::max());
		intercardinal = {};
	}
};

template <size_t BlockSize = 16>
struct rolling_cache_nodes
{
	uint64_t rolling;
	std::array<jps_cache_node, BlockSize> nodes;

	constexpr void init(uint64_t l_rolling) noexcept
	{
		if (rolling != l_rolling)
		{
			rolling = l_rolling;
			nodes = {};
		}
	}
};

template<warthog::memory::alloc::ByteFactory Factory, typename JpsJump = jump::jump_point_online>
class jps_cache_prune_expansion_policy : public jps_expansion_policy_base
{
	static_assert(InterSize >= 1, "InterSize must be at least 2.");

protected:
	static constexpr size_t cache_block_bits = 4;

	// factory layout:
	// Factory src
	// factory_pointer(src) -> cache_node_block_factory blocks
	// factory_pointer(src) -> cache_static_factory data
	
	using rolling_cache_block = rolling_cache_nodes<(1 << cache_block_bits)>;
	using cache_block_factory = warthog::memory::alloc::indexed_block_factory_type<rolling_cache_block,
		warthog::memory::alloc::make_factory_pointer<Factory>,
		warthog::memory::alloc::void_factory,
		0>;
	using cache_static_factory = warthog::memory::alloc::bump_factory<
		warthog::memory::alloc::make_factory_pointer<Factory>>;

public:
	/// @brief sets the policy to use with map
	/// @param map point to gridmap, if null map will need to be set later;
	///            otherwise sets map and creates a rotated gridmap.
	///            Use set_map to provide a map at a later stage.
	template <typename... FactoryArgs>
	jps_cache_prune_expansion_policy(warthog::domain::gridmap* map, FactoryArgs&& args...)
	    : jps_expansion_policy_base(map)
	{
		if (!factory_.setup(std::forward<FactoryArgs>(args)...)
			|| !cache_block_factory.setup(factory_)
			|| !cache_static_factory.setup(factory_))
		{
			WARTHOG_GCRIT("jps_cache_expansion_policy: failed to setup memory allocator");
			throw std::logic_error("failed to setup memory allocator");
		}
	}

	~jps_cache_prune_expansion_policy() = default;

	using jump_point = JpsJump;

	void map_changed();

	jps_cache_node* get_cache_node(grid_id id);

	void
	expand(
	    warthog::search::search_node* current,
	    warthog::search::search_problem_instance* pi) override;

	warthog::search::search_node*
	generate_start_node(warthog::search::search_problem_instance* pi) override;

	warthog::search::search_node*
	generate_target_node(
	    warthog::search::search_problem_instance* pi) override;

	size_t
	mem() override
	{
		return jps_expansion_policy_base::mem()
		    + (sizeof(jps_prune_expansion_policy)
		       - sizeof(jps_expansion_policy_base));
	}

	jump_point&
	get_jump_point() noexcept
	{
		return jpl_;
	}
	const jump_point&
	get_jump_point() const noexcept
	{
		return jpl_;
	}

	bool
	set_jump_limit(
	    jump::jump_distance limit
	    = std::numeric_limits<jump::jump_distance>::max()) noexcept
	    requires(InterLimit == 0)
	{
		if(limit < 1) return false;
		jump_limit_ = limit;
	}
	jump::jump_distance
	get_jump_limit() const noexcept
	    requires(InterLimit == 0)
	{
		return jump_limit_;
	}
	static constexpr jump::jump_distance
	get_jump_limit() noexcept
	    requires(InterLimit != 0)
	{
		return InterLimit < 0 ? std::numeric_limits<jump::jump_distance>::max()
		                      : static_cast<jump::jump_distance>(InterLimit);
	}

protected:
	void
	set_rmap_(domain::rotate_gridmap& rmap) override
	{
		jps_expansion_policy_base::set_rmap_(rmap);
		jpl_.set_map(rmap);
		map_width_ = rmap.map().width();
	}

private:
	JpsJump jpl_;
	point target_loc_   = {};
	grid_id target_id_  = {};
	uint32_t map_width_ = 0;
	jump::jump_distance jump_limit_
	    = std::numeric_limits<jump::jump_distance>::max();
	uint64_t rolling_reset_ = 0;
	Factory factory_;
	cache_block_factory block_factory_;
	cache_static_factory static_factory_;
	std::vector<intercardinal_jump_result> jump_build_;
};

template<warthog::memory::alloc::ByteFactory Factory, typename JpsJump>
void jps_cache_prune_expansion_policy<Factory, JpsJump>::map_changed()
{
	rolling_reset_ += 1;
	static_factory_.reclaim();
}

template<warthog::memory::alloc::ByteFactory Factory, typename JpsJump>
jps_cache_node* jps_cache_prune_expansion_policy<Factory, JpsJump>::get_cache_node(grid_id id)
{
	uint32_t block_id = (uint32_t)id >> cache_block_bits;
	uint32_t node_subid = (uint32_t)id & ((1u << cache_block_bits) - 1);
	assert(block_id < block_factory_.size());
	rolling_cache_block* block = block_factory_.get(block_id);\
	assert(block != nullptr);
	block->init(rolling_reset_);
	jps_cache_node* node = block->nodes[node_subid];
	if (node == nullptr) {
		node = block->nodes[node_subid] = reinterpret_cast<jps_cache_node*>(static_factory_.allocate(sizeof(jps_cache_node), alingof(jps_cache_node)));
		node->init();
	}
	assert(node != nullptr);
	return node;
}

template<warthog::memory::alloc::ByteFactory Factory, typename JpsJump>
void
jps_cache_prune_expansion_policy<Factory, JpsJump>::expand(
    warthog::search::search_node* current,
    warthog::search::search_problem_instance* instance)
{
	reset();

	// compute the direction of travel used to reach the current node.
	const grid_id current_id = grid_id(current->get_id());
	point loc                = rmap_.id_to_point(current_id);

	const direction dir_c = from_direction(
		grid_id(current->get_parent()), current_id, rmap_.map().width());
	const direction_id dir_cid = to_dir_id(dir_c);
	const direction_id target_d
		= warthog::grid::point_to_direction_id(loc, target_loc_);

	uint8_t succ_dirs;
	jps_cache_node* node;
	// if not using cache, node = nullptr
	if (current->id_ != instance->start_.id) [[likely]]
	{
		node = get_cache_node(current_id);
		succ_dirs = node->success_dir[dir_cid];
	} else {
		node = nullptr;
		succ_dirs = 0xFFu;
	}

	if (succ_dirs == 0xFFu)
	{
		// this dir_c has not being done (or is not cache)
		
		// compute correct success id
		domain::grid_pair_id pair_id{
			current_id, rmap_.rpoint_to_rid(rmap_.point_to_rpoint(loc))};
		assert(
			rmap_.map().get_label(get<grid_id>(pair_id))
			&& rmap_.rmap().get_label(
				grid_id(get<rgrid_id>(pair_id)))); // loc must be trav on map
		uint32_t c_tiles;
		map_->get_neighbours(current_id, (uint8_t*)&c_tiles);
		succ_dirs = static_cast<uint8_t>(compute_successors(dir_c, c_tiles));

		// cardinal directions
		::warthog::util::for_each_integer_sequence<std::integer_sequence<
			direction_id, NORTH_ID, EAST_ID, SOUTH_ID, WEST_ID>>([&](auto iv) {
			constexpr direction_id di = decltype(iv)::value;
			if(succ_dirs & warthog::grid::to_dir(di))
			{
				auto jump_result = jpl_.template jump_cardinal_next<di>(pair_id);
				if(jump_result > 0) // jump point
				{
					// successful jump
					pad_id node{static_cast<uint32_t>(
						current_id.id
						+ warthog::grid::dir_id_adj(di, map_width_)
							* jump_result)};
					assert(rmap_.map().get(node)); // successor must be traversable
					warthog::search::search_node* jp_succ = this->generate(node);
					add_neighbour(jp_succ, jump_result * warthog::DBL_ONE);
				}
			}
		});
		// intercardinal directions
		::warthog::util::for_each_integer_sequence<std::integer_sequence<
			direction_id, NORTHEAST_ID, NORTHWEST_ID, SOUTHEAST_ID, SOUTHWEST_ID>>(
			[&](auto iv) {
				constexpr direction_id di = decltype(iv)::value;
				if(succ_dirs & warthog::grid::to_dir(di))
				{
					const int32_t node_adj_ic
						= warthog::grid::dir_id_adj(di, map_width_);
					const int32_t node_adj_vert
						= warthog::grid::dir_id_adj_vert(di, map_width_);
					constexpr int32_t node_adj_hori
						= warthog::grid::dir_id_adj_hori(di);
					jump::intercardinal_jump_result res[InterSize];
					jump::jump_distance inter_total = 0;
					while(true)
					{
						auto [result_n, dist]
							= jpl_.template jump_intercardinal_many<di>(
								pair_id, res, InterSize, get_jump_limit());
						for(decltype(result_n) result_i = 0; result_i < result_n;
							++result_i) // jump point
						{
							// successful jump
							const jump::intercardinal_jump_result res_i
								= res[result_i];
							assert(res_i.inter > 0);
							const uint32_t node
								= current_id.id
								+ static_cast<uint32_t>(
									node_adj_ic * (inter_total + res_i.inter));
							const auto cost = warthog::DBL_ROOT_TWO
								* (inter_total + res_i.inter);
							assert(rmap_.map().get(
								pad_id{node})); // successor must be traversable
							if(res_i.hori > 0)
							{
								// horizontal
								const uint32_t node_j = node
									+ static_cast<uint32_t>(node_adj_hori
															* res_i.hori);
								const auto cost_j
									= cost + warthog::DBL_ONE * res_i.hori;
								assert(rmap_.map().get(
									pad_id{
										node_j})); // successor must be traversable
								warthog::search::search_node* jp_succ
									= this->generate(pad_id{node_j});
								add_neighbour(jp_succ, cost_j);
							}
							if(res_i.vert > 0)
							{
								// horizontal
								const uint32_t node_j = node
									+ static_cast<uint32_t>(node_adj_vert
															* res_i.vert);
								const auto cost_j
									= cost + warthog::DBL_ONE * res_i.vert;
								assert(rmap_.map().get(
									pad_id{
										node_j})); // successor must be traversable
								warthog::search::search_node* jp_succ
									= this->generate(pad_id{node_j});
								add_neighbour(jp_succ, cost_j);
							}
						}
						if(dist <= 0) // hit wall, break
							break;
						if constexpr(InterLimit < 0)
						{
							// repeat until all jump points are discovered
							inter_total += dist;
							loc          = loc + dist * dir_unit_point(di);
							pair_id      = rmap_.point_to_pair_id(loc);
						}
						else
						{
							// reach limit, push dia onto queue
							const uint32_t node = current_id.id
								+ static_cast<uint32_t>(node_adj_ic * dist);
							const auto cost = warthog::DBL_ROOT_TWO * dist;
							assert(rmap_.map().get(
								pad_id{node})); // successor must be traversable
							warthog::search::search_node* jp_succ
								= this->generate(pad_id{node});
							add_neighbour(jp_succ, cost);
							break;
						}
					}
				}
			});
	}



	// look for jump points in the direction of each natural
	// and forced neighbour
	uint32_t succ_dirs = compute_successors(dir_c, c_tiles);
	if(succ_dirs & static_cast<uint32_t>(warthog::grid::to_dir(target_d)))
	{
		// target in successor direction, check
		if(auto target_dist = jpl_.jump_target(pair_id, loc, target_loc_);
		   target_dist.second >= 0)
		{
			// target is visible, push
			warthog::search::search_node* jp_succ = this->generate(target_id_);
			add_neighbour(
			    jp_succ,
			    target_dist.first * warthog::DBL_ROOT_TWO
			        + target_dist.second * warthog::DBL_ONE);
			return; // no other successor required
		}
	}

	
}

template<typename JpsJump>
warthog::search::search_node*
jps_prune_expansion_policy<JpsJump>::
    generate_start_node(warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->start_) >= max_id) { return nullptr; }
	pad_id padded_id = pad_id(pi->start_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	target_id_ = grid_id(pi->target_);
	uint32_t x, y;
	rmap_.map().to_padded_xy(target_id_, x, y);
	target_loc_.x = static_cast<uint16_t>(x);
	target_loc_.y = static_cast<uint16_t>(y);
	return generate(padded_id);
}

template<typename JpsJump>
warthog::search::search_node*
jps_prune_expansion_policy<JpsJump>::
    generate_target_node(warthog::search::search_problem_instance* pi)
{
	uint32_t max_id = map_->width() * map_->height();
	if(static_cast<uint32_t>(pi->target_) >= max_id) { return nullptr; }
	pad_id padded_id = pad_id(pi->target_);
	if(map_->get_label(padded_id) == 0) { return nullptr; }
	return generate(padded_id);
}

} // namespace jps::search

#endif // JPS_SEARCH_JPS_CACHE_PRUNE_EXPANSION_POLICY_H
