cmake_minimum_required(VERSION 3.13)

# find include/jps -type f -name '*.h' | sort
target_sources(warthog_libjps PUBLIC
include/jps/forward.h
include/jps/jump/four_connected_jps_locator.h
include/jps/jump/offline_jump_point_locator.h
include/jps/jump/offline_jump_point_locator2.h
include/jps/jump/online_jump_point_locator.h
include/jps/jump/online_jump_point_locator2.h

include/jps/search/jps2_expansion_policy.h
include/jps/search/jps.h
include/jps/search/jps2plus_expansion_policy.h
include/jps/search/jps4c_expansion_policy.h
include/jps/search/jps_expansion_policy.h
include/jps/search/jpsplus_expansion_policy.h
)
