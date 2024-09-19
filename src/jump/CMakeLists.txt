cmake_minimum_required(VERSION 3.13)

# find -type f -name '*.cpp' | sed 's|./||' | sort
target_sources(warthog_libjps PRIVATE
offline_jump_point_locator.cpp
offline_jump_point_locator2.cpp
online_jump_point_locator.cpp
online_jump_point_locator2.cpp
)
