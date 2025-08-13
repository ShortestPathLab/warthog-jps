cmake_minimum_required(VERSION 3.13)

# ( cd src/search && echo *.cpp | sort )
target_sources(warthog_libjps PRIVATE
jps.cpp
)
