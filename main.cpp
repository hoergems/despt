#include "include/Planner.hpp"
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>
#include "include/OpptDummySolver.hpp"
#include "ABTOptions.hpp"

int main(int argc, char const* argv[])
{ 
	oppt::ProblemEnvironment problemEnvironment;
	int ret = problemEnvironment.setup<solvers::OpptDummySolver, oppt::ABTExtendedOptions>(argc, argv);
    if (ret != 0)
        return ret;
	oppt::OpptPlanner opptPlanner;
	opptPlanner.setProblemEnvironment(&problemEnvironment);
	char * argvTwo[0];
	return opptPlanner.RunPlanning(argc, argvTwo);
	return 0;
}