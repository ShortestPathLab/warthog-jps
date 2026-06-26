cmake_minimum_required(VERSION 3.13)

project(WarthogJPS
	VERSION 0.5.0
	LANGUAGES CXX C)

set_property(GLOBAL PROPERTY WARTHOG_warthog-jps ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include(cmake/warthog.cmake)
warthog_module_declare(warthog-core v0.5.0)
warthog_module(warthog-core)

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

