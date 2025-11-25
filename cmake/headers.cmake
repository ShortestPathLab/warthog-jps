cmake_minimum_required(VERSION 3.13)

# find include/jps -type f -name '*.h' | sort
target_sources(warthog_libjps PUBLIC
include/jps/forward.h

include/jps/domain/rotate_gridmap.h

include/jps/jump/block_online.h
include/jps/jump/jump.h
include/jps/jump/jump_point_offline.h
include/jps/jump/jump_point_online.h

include/jps/search/jps_expansion_policy_base.h
include/jps/search/jps_expansion_policy.h
include/jps/search/jps.h
include/jps/search/jps_prune_expansion_policy.h
)
