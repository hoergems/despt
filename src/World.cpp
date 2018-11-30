#include "include/World.hpp"
#include "include/State.hpp"
#include "include/Model.hpp"
#include "ABTOptions.hpp"
#include <oppt/robotHeaders/ActionSpaceDiscretizer.hpp>

namespace oppt {
DespotWorld::DespotWorld(despot::DSPOMDP *model):
	model_(model) {

}

DespotWorld::~DespotWorld() {

}

bool DespotWorld::Connect() {

}

despot::State* DespotWorld::Initialize() {
	ERROR("Implement Initialize");
}

despot::State* DespotWorld::GetCurrentState() const {
	return currentDespotState_.get();
}

bool DespotWorld::ExecuteAction(despot::ACT_TYPE action, despot::OBS_TYPE& obs) {
	if (!problemEnvironment_)
		ERROR("No problem environment");
	cout << "action number: " << action << endl;
	cout << "execute action: " << *(actions_[action].get()) << endl;
	auto robotEnvironment = problemEnvironment_->getRobotExecutionEnvironment();

	PropagationRequestSharedPtr propagationRequest(new PropagationRequest);
	propagationRequest->currentState = static_cast<DespotState *>(currentDespotState_.get())->getOpptState();
	propagationRequest->action = actions_[action];

	PropagationResultSharedPtr propagationResult = robotEnvironment->getRobot()->propagateState(propagationRequest);

	static_cast<DespotState *>(currentDespotState_.get())->setOpptState(propagationResult->nextState);

	ObservationRequestSharedPtr observationRequest(new ObservationRequest);
	observationRequest->currentState = propagationResult->nextState;
	observationRequest->action = propagationRequest->action;

	ObservationResultSharedPtr observationResult =
	    robotEnvironment->getRobot()->makeObservationReport(observationRequest);

	auto obsMap = static_cast<DespotModel *>(model_)->getObservationMap();
	auto res = obsMap->emplace(observationResult->observation, obsMap->getMap()->size(), true);
	obs = res.first->second;

	bool terminal = robotEnvironment->isTerminal(propagationResult);
	if (terminal)
		WARNING("Terminal state reached");
	problemEnvironment_->updateViewer(propagationResult->nextState); 
	return terminal;
}

void DespotWorld::setProblemEnvironment(ProblemEnvironment *problemEnvironment) {
	if (!problemEnvironment)
		ERROR("problemEnvironment is null");
	problemEnvironment_ = problemEnvironment;
	auto currentState = problemEnvironment_->getRobotExecutionEnvironment()->sampleInitialState();
	currentDespotState_ = std::make_unique<DespotState>();
	static_cast<DespotState *>(currentDespotState_.get())->setOpptState(currentState);
	setDiscretizedActions();
}

ProblemEnvironment *DespotWorld::getProblemEnvironment() const {
	return problemEnvironment_;
}

void DespotWorld::setDiscretizedActions() {
	auto options = static_cast<const ABTExtendedOptions *>(problemEnvironment_->getOptions());
	auto actionSpace = problemEnvironment_->getRobotExecutionEnvironment()->getRobot()->getActionSpace();
	auto actionSpaceDiscretizer = actionSpace->getActionSpaceDiscretizer();
	if (!actionSpaceDiscretizer) {
		actionSpaceDiscretizer = std::make_shared<oppt::ActionSpaceDiscretizer>(actionSpace);
		if (options->actionDiscretization.size() > 0) {
			actionSpaceDiscretizer = std::make_unique<CustomActionSpaceDiscretizer>(actionSpace, options->actionDiscretization);
		}
	}

	actions_ = actionSpaceDiscretizer->getAllActionsInOrder(options->numInputStepsActions);
	LOGGING("ACTIONS SIZE: " + std::to_string(actions_.size()));
}

}