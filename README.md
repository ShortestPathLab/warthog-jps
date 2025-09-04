# JPS Library

Warthog is an optimised C++ library for pathfinding search.
Warthog-JPS is an extension to Warthog with JPS support.
See [warthog-core](https://github.com/ShortestPathLab/warthog-core) for more details on Warthog setup.

# Using JPS

## Compile

Program `warthog-jps` uses cmake for the compile process.
Simply run the following code below:

```
cmake -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build
./build/warthog-jps --alg jps --scen example.scen
```

JPS requires `warthog-core` repo to compile, and by default will fetch the repo.

## Use in project

If elements of warthog is used in a project, follow guides from `warthog-core` to set up the project structure.

# Resources

- [Moving AI Lab](https://movingai.com/): pathfinding benchmark and tools
- [Posthoc](https://posthoc-app.pathfinding.ai/): visualiser and debugger for pathfinding
- [Pathfinding Benchmarks](https://benchmarks.pathfinding.ai/): git repo for benchmarks
