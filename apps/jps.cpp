// warthog.cpp
//
// Pulls together a variety of different algorithms
// for pathfinding on grid graphs.
//
// @author: dharabor
// @created: 2016-11-23
//

#include <jps/search/jps.h>
#include <jps/search/jps2_expansion_policy.h>
#include <jps/search/jps2plus_expansion_policy.h>
#include <jps/search/jps4c_expansion_policy.h>
#include <jps/search/jps_expansion_policy.h>
#include <jps/search/jpsplus_expansion_policy.h>
#include <warthog/constants.h>
#include <warthog/domain/gridmap.h>
#include <warthog/heuristic/octile_heuristic.h>
#include <warthog/search/unidirectional_search.h>
#include <warthog/util/pqueue.h>
#include <warthog/util/scenario_manager.h>
#include <warthog/util/timer.h>

#include "cfg.h"
#include "config.h"
#include <getopt.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <unordered_map>

// #include "time_constraints.h"

namespace
{
// check computed solutions are optimal
int checkopt = 0;
// print debugging info during search
int verbose = 0;
// display program help on startup
int print_help = 0;

void
help(std::ostream& out)
{
	out << "warthog version " << WARTHOG_VERSION << "\n";
	out << "==> manual <==\n"
	    << "This program solves/generates grid-based pathfinding "
	       "problems using the\n"
	    << "map/scenario format from the 2014 Grid-based Path Planning "
	       "Competition\n\n";

	out << "The following are valid parameters for SOLVING instances:\n"
	    << "\t--alg [alg] (required)\n"
	    << "\t--scen [scen file] (required) \n"
	    << "\t--map [map file] (optional; specify this to override map "
	       "values in scen file) \n"
	    << "\t--costs [costs file] (required if using a weighted "
	       "terrain algorithm)\n"
	    << "\t--checkopt (optional; compare solution costs against "
	       "values in the scen file)\n"
	    << "\t--verbose (optional; prints debugging info when compiled "
	       "with debug symbols)\n"
	    << "Invoking the program this way solves all instances in [scen "
	       "file] with algorithm [alg]\n"
	    << "Currently recognised values for [alg]:\n"
	    << "\tjps, jps+, jps2, jps2+\n";
	// << ""
	// << "The following are valid parameters for GENERATING instances:\n"
	// << "\t --gen [map file (required)]\n"
	// << "Invoking the program this way generates at random 1000 valid
	// problems for \n"
	// << "gridmap [map file]\n";
}

bool
check_optimality(
    warthog::search::solution& sol, warthog::util::experiment* exp)
{
	uint32_t precision = 2;
	double epsilon = (1.0 / (int)pow(10, precision)) / 2;
	double delta = fabs(sol.sum_of_edge_costs_ - exp->distance());

	if(fabs(delta - epsilon) > epsilon)
	{
		std::stringstream strpathlen;
		strpathlen << std::fixed << std::setprecision(exp->precision());
		strpathlen << sol.sum_of_edge_costs_;

		std::stringstream stroptlen;
		stroptlen << std::fixed << std::setprecision(exp->precision());
		stroptlen << exp->distance();

		std::cerr << std::setprecision(exp->precision());
		std::cerr << "optimality check failed!" << std::endl;
		std::cerr << std::endl;
		std::cerr << "optimal path length: " << stroptlen.str()
		          << " computed length: ";
		std::cerr << strpathlen.str() << std::endl;
		std::cerr << "precision: " << precision << " epsilon: " << epsilon
		          << std::endl;
		std::cerr << "delta: " << delta << std::endl;
		exit(1);
	}
	return true;
}

template<typename Search>
void
run_experiments(
    Search& algo, std::string alg_name,
    warthog::util::scenario_manager& scenmgr, bool verbose, bool checkopt,
    std::ostream& out)
{
	std::cout << "id\talg\texpanded\tgenerated\treopen\tsurplus\theapops"
	          << "\tnanos\tplen\tpcost\tscost\tmap\n";
	for(unsigned int i = 0; i < scenmgr.num_experiments(); i++)
	{
		warthog::util::experiment* exp = scenmgr.get_experiment(i);

		warthog::pack_id startid{
		    exp->starty() * exp->mapwidth() + exp->startx()};
		warthog::pack_id goalid{exp->goaly() * exp->mapwidth() + exp->goalx()};
		warthog::search::problem_instance pi(startid, goalid, verbose);
		warthog::search::search_parameters par;
		warthog::search::solution sol;

		algo.get_path(&pi, &par, &sol);

		out << i << "\t" << alg_name << "\t" << sol.met_.nodes_expanded_
		    << "\t" << sol.met_.nodes_generated_ << "\t"
		    << sol.met_.nodes_reopen_ << "\t" << sol.met_.nodes_surplus_
		    << "\t" << sol.met_.heap_ops_ << "\t"
		    << sol.met_.time_elapsed_nano_.count() << "\t"
		    << (sol.path_.size() - 1) << "\t" << sol.sum_of_edge_costs_ << "\t"
		    << exp->distance() << "\t" << scenmgr.last_file_loaded()
		    << std::endl;

		if(checkopt) { check_optimality(sol, exp); }
	}
}

template<typename ExpansionPolicy>
void
run_jps(
    warthog::util::scenario_manager& scenmgr, std::string mapname,
    std::string alg_name)
{
	warthog::domain::gridmap map(mapname.c_str());
	ExpansionPolicy expander(&map);
	warthog::heuristic::octile_heuristic heuristic(map.width(), map.height());
	warthog::util::pqueue_min open;

	warthog::search::unidirectional_search jps(&heuristic, &expander, &open);

	run_experiments(jps, alg_name, scenmgr, verbose, checkopt, std::cout);
	std::cerr << "done. total memory: " << jps.mem() + scenmgr.mem() << "\n";
}

} // namespace

int
main(int argc, char** argv)
{
	// parse arguments
	warthog::util::param valid_args[]
	    = {{"alg", required_argument, 0, 1},
	       {"scen", required_argument, 0, 0},
	       {"map", required_argument, 0, 1},
	       // {"gen", required_argument, 0, 3},
	       {"help", no_argument, &print_help, 1},
	       {"checkopt", no_argument, &checkopt, 1},
	       {"verbose", no_argument, &verbose, 1},
	       {"costs", required_argument, 0, 1},
	       {0, 0, 0, 0}};

	warthog::util::cfg cfg;
	cfg.parse_args(argc, argv, "a:b:c:def", valid_args);

	if(argc == 1 || print_help)
	{
		help(std::cout);
		exit(0);
	}

	std::string sfile = cfg.get_param_value("scen");
	std::string alg = cfg.get_param_value("alg");
	// std::string gen = cfg.get_param_value("gen");
	std::string mapfile = cfg.get_param_value("map");
	std::string costfile = cfg.get_param_value("costs");

	// if(gen != "")
	// {
	// 	warthog::util::scenario_manager sm;
	// 	warthog::domain::gridmap gm(gen.c_str());
	// 	sm.generate_experiments(&gm, 1000) ;
	// 	sm.write_scenario(std::cout);
	//     exit(0);
	// }

	// running experiments
	if(alg == "" || sfile == "")
	{
		help(std::cout);
		exit(0);
	}

	// load up the instances
	warthog::util::scenario_manager scenmgr;
	scenmgr.load_scenario(sfile.c_str());

	if(scenmgr.num_experiments() == 0)
	{
		std::cerr << "err; scenario file does not contain any instances\n";
		exit(0);
	}

	// the map filename can be given or (default) taken from the scenario file
	if(mapfile == "")
	{
		// first, try to load the map from the scenario file
		mapfile = scenmgr.get_experiment(0)->map().c_str();
		if(!std::filesystem::exists(std::filesystem::path(mapfile)))
		{
			// else, look for the map in the current directory
			mapfile = std::filesystem::path(mapfile).filename();
			if(!std::filesystem::exists(std::filesystem::path(mapfile)))
			{
				// else, try to infer the map name from the scenario filename
				std::filesystem::path p(sfile);
				mapfile = std::filesystem::path(sfile).replace_extension("");
				if(!std::filesystem::exists(std::filesystem::path(mapfile)))
				{
					std::cerr << "could not locate a corresponding map file\n";
					help(std::cout);
					exit(0);
				}
			}
		}
	}
	std::cerr << "mapfile=" << mapfile << std::endl;

	using namespace jps::search;
	if(alg == "jps") { run_jps<jps_expansion_policy>(scenmgr, mapfile, alg); }
	else if(alg == "jps+")
	{
		run_jps<jpsplus_expansion_policy>(scenmgr, mapfile, alg);
	}
	else if(alg == "jps2")
	{
		run_jps<jps2_expansion_policy>(scenmgr, mapfile, alg);
	}
	else if(alg == "jps2+")
	{
		run_jps<jps2plus_expansion_policy>(scenmgr, mapfile, alg);
	}
	else
	{
		std::cerr << "err; invalid search algorithm: " << alg << "\n";
		return 1;
	}

	return 0;
}
