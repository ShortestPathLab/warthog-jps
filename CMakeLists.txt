cmake_minimum_required(VERSION 3.13)

project(WarthogJPS
	VERSION 0.2.0
	LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include(cmake/submodule.cmake)

add_library(warthog_libjps)
add_library(warthog::libjps ALIAS warthog_libjps)
target_include_directories(warthog_libjps PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(warthog_libjps PUBLIC warthog::core)
include(cmake/headers.cmake)

add_executable(warthog_jps)
add_executable(warthog::jps ALIAS warthog_jps)
set_target_properties(warthog_jps PROPERTIES OUTPUT_NAME "warthog-jps")
target_link_libraries(warthog_jps PUBLIC warthog::libjps)

add_subdirectory(src)
add_subdirectory(apps)

warthog_submodule(warthog-core)
