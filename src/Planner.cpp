#include "include/Planner.hpp"
#include <oppt/opptCore/core.hpp>
#include "include/Model.hpp"
#include "include/World.hpp"
#include "include/State.hpp"
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
	std::string resultsDir = problemEnvironment_->getOptions()->logPath;
	if (!oppt::createDir(resultsDir)) {
		ERROR("Results directory couldn't be created: " + resultsDir);
	}

	std::string finalLogFilePath = problemEnvironment_->getLogFilePath(resultsDir);
	if (!oppt::fileExists(finalLogFilePath)) {
		os_ = std::ofstream(finalLogFilePath,
		                    std::ios_base::app | std::ios_base::out);
	}

	return despotModel;
}

void OpptPlanner::PlanningLoop(despot::Solver*& solver, despot::World* world, despot::Logger* logger) {
	FloatType discountFactor = problemEnvironment_->getOptions()->discountFactor;
	FloatType totalReward = 0.0;
	FloatType immediateReward = 0.0;
	for (int i = 0; i < despot::Globals::config.sim_len; i++) {
		bool terminal = RunStep(solver, world, logger);
		FloatType immediateReward = static_cast<DespotWorld*>(world)->getImmediateReward();
		os_ << "t = " << i << endl;
		os_ << "IMMEDIATE_REWARD: " << immediateReward << endl;
		os_ << "DISCOUNT_FACTOR: " << std::pow(discountFactor, i) << endl;
		os_ << "DISCOUNTED_REWARD: " << std::pow(discountFactor, i) * immediateReward << endl;
		WARNING("terminal: " + std::to_string(terminal));
		totalReward += std::pow(discountFactor, i) * immediateReward;
		cout << "total discounted reward: " << totalReward << endl;
		if (terminal) {
			//if (immediateReward == problemEnvironment_->getRobotExecutionEnvironment()->getRewardPlugin())
			break;
		}

		if (i >= problemEnvironment_->getOptions()->nSimulationSteps) {
			LOGGING("OUT OF STEPS");
			break;
		}

	}

	os_ << "Total discounted reward: " << totalReward << endl;
	if (immediateReward == problemEnvironment_->getRobotExecutionEnvironment()->getRewardPlugin()->getMinMaxReward().second) {
		os_ << "Run successful: True\n";
	} else {
		os_ << "Run successful: False\n";
	}
}

bool OpptPlanner::RunStep(despot::Solver* solver, despot::World* world, despot::Logger* logger) {
	logger->CheckTargetTime();

	double step_start_t = despot::get_time_second();

	double start_t = despot::get_time_second();
	despot::ACT_TYPE action = solver->Search().action;
	double end_t = despot::get_time_second();
	double search_time = (end_t - start_t);
	logi << "[RunStep] Time spent in " << typeid(*solver).name()
	     << "::Search(): " << search_time << endl;

	despot::OBS_TYPE obs;
	start_t = despot::get_time_second();
	bool terminal = world->ExecuteAction(action, obs);
	end_t = despot::get_time_second();
	double execute_time = (end_t - start_t);
	logi << "[RunStep] Time spent in ExecuteAction(): " << execute_time << endl;

	start_t = despot::get_time_second();
	solver->BeliefUpdate(action, obs);
	end_t = despot::get_time_second();
	double update_time = (end_t - start_t);
	logi << "[RunStep] Time spent in Update(): " << update_time << endl;

	auto currentState = static_cast<DespotState *>(world->GetCurrentState())->getOpptState();
	auto model = static_cast<DespotModel *>(static_cast<DespotWorld *>(world)->getModel());
	VectorRobotStatePtr particles = static_cast<Belief *>(model->getBelief())->getOpptParticles();

	problemEnvironment_->updateViewer(currentState, particles);

	return logger->SummarizeStep(step_++, round_, terminal, action, obs,
	                             step_start_t);

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
	despot::Globals::config.sim_len = options->nSimulationSteps;
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