cmake_minimum_required(VERSION 3.13)

# ( cd src/search && echo *.cpp | sort )
target_sources(warthog_libjps PRIVATE
jps2_expansion_policy.cpp
jps.cpp
jps2plus_expansion_policy.cpp
jps4c_expansion_policy.cpp
jps_expansion_policy.cpp
jpsplus_expansion_policy.cpp
)
