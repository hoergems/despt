#ifndef _DESPOT_OPPT_STATE_HPP_
#define _DESPOT_OPPT_STATE_HPP_
#include <oppt/opptCore/core.hpp>
#include <despot/interface/pomdp.h>

namespace oppt {

class DespotState: public despot::State {
public:
	DespotState();

	DespotState(int &stateId, int &weight);

	virtual ~DespotState();

	void setOpptState(RobotStateSharedPtr &opptState);

	RobotStateSharedPtr getOpptState() const;

	void print(std::ostream& out) const;

private:
	RobotStateSharedPtr opptState_ = nullptr;

};

}

#endif