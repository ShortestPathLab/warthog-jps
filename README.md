# JPS Library

Warthog is an optimised C++ library for pathfinding search.
Warthog-JPS is an extension to Warthog with JPS support.
See [warthog-core](https://github.com/ShortestPathLab/warthog-core) for more details on Warthog setup.

If you find this work useful, please cite our paper as follows:

	@article{Harabor_Grastien_2014,
	title={Improving Jump Point Search},
	volume={24},
	url={https://ojs.aaai.org/index.php/ICAPS/article/view/13633},
	DOI={10.1609/icaps.v24i1.13633},
	number={1},
	journal={Proceedings of the International Conference on Automated Planning and Scheduling},
	author={Harabor, Daniel and Grastien, Alban},
	year={2014},
	pages={128-135}
	}

# Using JPS

## Compile

Program `warthog-jps` uses cmake for the compile process.
Simply run the following code below:

```
cmake -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build
./build/warthog-jps --alg jps --scen example/arena2.map.scen
```

The example map is from the MovingAI benchmarks under Dragon Age Origins.
Use the `build.sh` or `run.sh` for quick compile/usage of this scenario.

For more evaluation data, please refer to existing benchmarks from the community, which offer a range of different challenges across a variety of maps/domains; links found in the resource section.

JPS requires `warthog-core` repo to compile, and by default will fetch the repo.

## JPS Variants

JPS comes with several variants.
All variants use block-based symmetry breaking for finding jump-point locations,
making them at least JPS (B).
The list is as follows:

- `jps` JPS (B) like original with block-based symmetry
- `jpsP` or `jps2` JPS (B+P) with intercardinal pruning
- `jps+` JPS+ (B) is original with offline jump points
- `jpsP+` or `jps2+` JPS+ (B+P) offline with pruning

These jump points are found in namespace `jps::jump`, while `jps::search` uses provided `jps::jump` locators
to produce successors used in warthog-core.

The algorithms provided to `--alg` parameter, along with their `jps::search` and `jps::jump` classes presented
in the table below:

| `--alg`            | `jps::search`                  | `jps::jump`            |
|--------------------|--------------------------------|------------------------|
| `jps`              | `jps_expansion_policy<>`       | `jump_point_online`    |
| `jpsP` or `jps2`   | `jps_prune_expansion_policy<>` | `jump_point_online`    |
| `jps+`             | `jps_expansion_policy`         | `jump_point_offline<>` |
| `jpsP+` or `jps2+` | `jps_prune_expansion_policy<>` | `jump_point_offline<>` |

## Use in project

If elements of warthog is used in a project, follow guides from `warthog-core` to set up the project structure.

# Resources

- [Moving AI Lab](https://movingai.com/): pathfinding benchmark and tools
- [Posthoc](https://posthoc-app.pathfinding.ai/): visualiser and debugger for pathfinding
- [Pathfinding Benchmarks](https://benchmarks.pathfinding.ai/): git repo for benchmarks
