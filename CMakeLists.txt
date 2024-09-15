cmake_minimum_required(VERSION 3.13)

project(WarthogJPS
	VERSION 0.1.0
	LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include(cmake/submodule.cmake)

add_library(warthog_jps)
add_library(warthog::libjps ALIAS warthog_jps)
target_include_directories(warthog_jps PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(warthog_jps PUBLIC warthog::core)

add_executable(warthog_jps_app)
add_executable(warthog::jps ALIAS warthog_jps_app)
set_target_properties(warthog_jps_app PROPERTIES OUTPUT_NAME "jps")
target_link_libraries(warthog_jps_app PUBLIC warthog::libjps)

add_subdirectory(src)
add_subdirectory(apps)

warthog_submodule(warthog-core)
