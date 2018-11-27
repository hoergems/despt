#ifndef _OPPT_DUMMY_SOLVER_HPP_
#define _OPPT_DUMMY_SOLVER_HPP_
#include <oppt/solver/solver.hpp>

using namespace oppt;
namespace solvers {

class OpptDummySolver: public Solver {
public:
	OpptDummySolver():
		Solver() {

	}

	/**
     * Virtual destructor
     */
    virtual ~OpptDummySolver() {}

    /**
     * @brief Performs initial setup operations
     */
    virtual void setup() override {

    }

    /**
     * @brief Perform reset operations. This method is called after each simulation run
     */
    virtual bool reset() override {
    	return true;

    }

    /**
     * @brief Improve the POMDP policy starting from the current belief.
     * @param timeout The maximum time (in milliseconds) the solver should take to improve the policy
     *
     * @return true if the policy improvement was successful
     */
    virtual bool improvePolicy(const FloatType &timeout) override {
    	return true;
    }

    /**
     * @brief Get the next action the robot has to execute according to the calculated
     * policy.
     *
     * @return A shared pointer to the Action that is going to be executed.
     * If there's no action available, a nullptr should be returned
     */
    virtual ActionSharedPtr getNextAction() override {
    	return nullptr;
    }

    /**
     * @brief Update the belief, based on the action taken and observation received
     * @param action The action that was executed
     * @param observation The observation that was percieved
     * @param allowTerminalStates If true, the next belief is allowed to have terminal states
     *
     * @return true if the belief update was successful
     */
    virtual bool updateBelief(const ActionSharedPtr& action,
                              const ObservationSharedPtr& observation,
                              const bool &allowTerminalStates = false) override {
    	return true;
    }

    oppt::HeuristicPlugin *getHeuristicPlugin() const {
    	return heuristicPlugin_.get();
    }   

};

}


#endif