#ifndef _DESPOT_OPPT_PLANNER_HPP_
#define _DESPOT_OPPT_PLANNER_HPP_
#include <despot/planner.h>
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>

namespace oppt {

class OpptPlanner: public despot::Planner {

public:
	OpptPlanner();

	virtual ~OpptPlanner();

	virtual std::string ChooseSolver() override;

	virtual despot::DSPOMDP* InitializeModel(despot::option::Option* options) override;

	virtual despot::World* InitializeWorld(std::string&  world_type, despot::DSPOMDP* model, despot::option::Option* options) override;

	virtual void InitializeDefaultParameters() override;

	/**
	 * Run and evaluate POMDP planning for a given number of rounds
	 */
	virtual int RunEvaluation(int argc, char* argv[]) override;

	virtual void PlanningLoop(despot::Solver*& solver, despot::World* world, despot::Logger* logger) override;

	virtual bool RunStep(despot::Solver* solver, despot::World* world, despot::Logger* logger) override;

	/**
	 * Evaluate the planner by repeating a test problem for multiple trials
	 * Overwrite this function to customize your evaluation pipeline
	 */
	virtual void EvaluationLoop(despot::DSPOMDP *model, despot::World* world, despot::Belief* belief,
	                            std::string belief_type, despot::Solver *&solver, despot::Logger *logger,
	                            despot::option::Option *options, clock_t main_clock_start, int num_runs,
	                            int start_run) override;

	void setProblemEnvironment(ProblemEnvironment *problemEnvironment);

private:
	ProblemEnvironment *problemEnvironment_ = nullptr;

	std::ofstream os_;

};

}

#endif