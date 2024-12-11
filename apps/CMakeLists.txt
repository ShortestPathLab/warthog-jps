cmake_minimum_required(VERSION 3.13)

configure_file(config.h.in config.h @ONLY)
target_sources(warthog_jps PRIVATE jps.cpp cfg.cpp)
target_sources(warthog_jps PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/config.h cfg.h)
target_include_directories(warthog_jps PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
