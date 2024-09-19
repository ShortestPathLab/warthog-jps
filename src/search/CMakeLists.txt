cmake_minimum_required(VERSION 3.13)

# find -type f -name '*.cpp' | sed 's|./||' | sort
target_sources(warthog_libjps PRIVATE
jps.cpp
jps2_expansion_policy.cpp
jps2plus_expansion_policy.cpp
jps4c_expansion_policy.cpp
jps_expansion_policy.cpp
jpsplus_expansion_policy.cpp
)
