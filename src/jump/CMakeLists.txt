cmake_minimum_required(VERSION 3.13)

# ( cd src/jump && echo *.cpp | sort )
target_sources(warthog_libjps PRIVATE
four_connected_jps_locator.cpp
offline_jump_point_locator.cpp
offline_jump_point_locator2.cpp
online_jump_point_locator.cpp
online_jump_point_locator2.cpp
)
