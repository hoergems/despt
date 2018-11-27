#ifndef _DESPOT_OPPT_WORLD_HPP_
#define _DESPOT_OPPT_WORLD_HPP_
#include <despot/interface/world.h>
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>

namespace oppt {
class DespotWorld: public despot::World {
public:
    DespotWorld(despot::DSPOMDP *model);

	virtual ~DespotWorld();
	/*Establish connection with the external system*/
	virtual bool Connect() override;
	/*Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable*/
	virtual despot::State* Initialize() override;
	/*Get the state of the system (only applicable for simulators or POMDP world)*/
	virtual despot::State* GetCurrentState() const override;
	/*Send action to be executed by the system, receive observations terminal signals from the system*/
	virtual bool ExecuteAction(despot::ACT_TYPE action, despot::OBS_TYPE& obs) override;

	void setProblemEnvironment(ProblemEnvironment *problemEnvironment);

	ProblemEnvironment *getProblemEnvironment() const;

private:
	ProblemEnvironment *problemEnvironment_ = nullptr;

	std::unique_ptr<despot::State> currentDespotState_ = nullptr;

	std::vector<ActionSharedPtr> actions_;

	despot::DSPOMDP *model_ = nullptr;

private:

	void setDiscretizedActions();

};

}

#endif