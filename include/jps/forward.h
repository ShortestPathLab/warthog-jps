#ifndef JPS_FORWARD_H
#define JPS_FORWARD_H

//
// jps/forward.h
//
// Global namespace types include.
//
// @author Ryan Hechenberger
// @created 2025-11-20
//

#include <warthog/constants.h>
#include <warthog/defines.h>
#include <warthog/domain/grid.h>
#include <warthog/forward.h>

namespace jps
{

using namespace ::warthog::grid;
using ::warthog::pad_id;
// using jps_id = grid_id;
struct rmap_id_tag
{ };
using rgrid_id = warthog::identity_base<rmap_id_tag, grid_id::id_type>;
using warthog::cost_t;

} // namespace jps

#include <jps/jump/jump.h>
#include <jps/search/jps.h>

#endif // JPS_SEARCH_FORWARD_H
