#ifndef JPS_FORWARD_H
#define JPS_FORWARD_H

#include <vector>
#include <warthog/constants.h>
#include <warthog/defines.h>
#include <warthog/domain/grid.h>
#include <warthog/forward.h>

namespace jps
{

using namespace warthog::grid;
using ::warthog::pad_id;
using jps_id = grid_id;
struct rmap_id_tag
{ };
using rgrid_id = warthog::identity_base<rmap_id_tag, jps_id::id_type>;
using warthog::cost_t;

using vec_jps_id   = std::vector<jps_id>;
using vec_jps_cost = std::vector<cost_t>;

enum class JpsFeature : uint8_t
{
	DEFAULT             = 0, // uses block-based jumping
	PRUNE_INTERCARDINAL = 1 << 0,
	STORE_CARDINAL_JUMP = 1 << 1, // if not PRUNE_INTERCARDINAL, then store
	                              // cardinal results in intercandial jump
};
inline JpsFeature
operator|(JpsFeature a, JpsFeature b) noexcept
{
	return static_cast<JpsFeature>(
	    static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

} // namespace jps

#endif // JPS_SEARCH_FORWARD_H
