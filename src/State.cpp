#include "include/State.hpp"

using std::cout;
using std::endl;

namespace oppt {
DespotState::DespotState():
	despot::State() {

}

DespotState::DespotState(int &stateId, int &weight):
	despot::State(stateId, weight)
{

}

DespotState::~DespotState() {

}

void DespotState::setOpptState(RobotStateSharedPtr &opptState) {
	opptState_ = opptState;
}

RobotStateSharedPtr DespotState::getOpptState() const {
	return opptState_;
}

void DespotState::setPreviousState(RobotStateSharedPtr previousState) {
	previousState_ = previousState;
}

RobotState *DespotState::getPreviousState() const {
	if (previousState_)
		return previousState_.get();
	return nullptr;
}

void DespotState::print(std::ostream& out) const {
	if (!opptState_)
		ERROR("OpptState is null");
	out << *(opptState_.get()) << endl;
}

}