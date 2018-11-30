#include "include/Planner.hpp"
#include <oppt/opptCore/core.hpp>
#include "include/Model.hpp"
#include "include/World.hpp"
#include "ABTOptions.hpp"

namespace oppt {
OpptPlanner::OpptPlanner() {	

}

OpptPlanner::~OpptPlanner() {

}

void OpptPlanner::setProblemEnvironment(ProblemEnvironment *problemEnvironment) {
	problemEnvironment_ = problemEnvironment;
}

std::string OpptPlanner::ChooseSolver() { // Specify the solver used in the planner to be DESPOT
	return "POMCP";
}

despot::DSPOMDP* OpptPlanner::InitializeModel(despot::option::Option* options) {
	DespotModel *despotModel = new DespotModel();
	static_cast<DespotModel *>(despotModel)->setProblemEnvironment(problemEnvironment_);
	return despotModel;
	ERROR("Initialze model");

}


despot::World* OpptPlanner::InitializeWorld(std::string& world_type, despot::DSPOMDP* model, despot::option::Option* options) {
	despot::World *world = new DespotWorld(model);
	static_cast<DespotWorld *>(world)->setProblemEnvironment(problemEnvironment_);
	world_type = "myWorld";	
	return world;	
}

void OpptPlanner::InitializeDefaultParameters()  { // Specify DESPOT parameters for the particular problem
	auto options = static_cast<const ABTExtendedOptions *>(problemEnvironment_->getOptions());
	despot::Globals::config.time_per_move = options->stepTimeout / 1000.0;
	despot::Globals::config.pruning_constant = 0.01;
	despot::Globals::config.num_scenarios = options->numScenarios;
	despot::Globals::config.search_depth = options->searchDepth;
	despot::Globals::config.discount = options->discountFactor;
}

/**
 * Run and evaluate POMDP planning for a given number of rounds
 */
int OpptPlanner::RunEvaluation(int argc, char* argv[]) {

}

/**
 * Evaluate the planner by repeating a test problem for multiple trials
 * Overwrite this function to customize your evaluation pipeline
 */
void OpptPlanner::EvaluationLoop(despot::DSPOMDP *model, despot::World* world, despot::Belief* belief,
                                 std::string belief_type, despot::Solver *&solver, despot::Logger *logger,
                                 despot::option::Option *options, clock_t main_clock_start, int num_runs,
                                 int start_run) {

}


}