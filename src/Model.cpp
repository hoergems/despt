#include "include/Model.hpp"
#include "include/State.hpp"
#include "include/Belief.hpp"
#include "include/ObservationMap.hpp"
#include "include/Bound.hpp"
#include "include/POMCPPrior.hpp"
#include "ABTOptions.hpp"
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>
#include <oppt/robotHeaders/ActionSpaceDiscretizer.hpp>


namespace oppt {

DespotModel::DespotModel():
	despot::DSPOMDP() {

}

DespotModel::~DespotModel() {

}

void DespotModel::setProblemEnvironment(oppt::ProblemEnvironment *problemEnvironment) {
	problemEnvironment_ = problemEnvironment;
	setDiscretizedActions();

	observationMap_ =
	    std::make_unique<ObservationMap>(static_cast<const ABTExtendedOptions *>(problemEnvironment_->getOptions())->maxObservationDistance);
}

void DespotModel::setObservationMap(ObservationMapPtr observationMap) {
	observationMap_ = std::move(observationMap);
}

ObservationMap *DespotModel::getObservationMap() const {
	return observationMap_.get();
}

void DespotModel::setDiscretizedActions() {
	auto options = static_cast<const ABTExtendedOptions *>(problemEnvironment_->getOptions());
	auto actionSpace = problemEnvironment_->getRobotPlanningEnvironment()->getRobot()->getActionSpace();
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

oppt::ProblemEnvironment *DespotModel::getProblemEnvironment() const {
	return problemEnvironment_;
}

bool DespotModel::Step(despot::State& state, double random_num, despot::ACT_TYPE action,
                       double& reward, despot::OBS_TYPE& obs) const {
	stepCounter_++;
	auto options = static_cast<const ABTExtendedOptions *>(problemEnvironment_->getOptions());
	RandomEnginePtr randomEngine(new RandomEngine((unsigned) RAND_MAX * random_num));

	auto robotEnvironment = problemEnvironment_->getRobotPlanningEnvironment();
	auto robot = problemEnvironment_->getRobotPlanningEnvironment()->getRobot();
	robot->getTransitionPlugin()->getErrorDistribution()->setRandomEngine(randomEngine);
	robot->getObservationPlugin()->getErrorDistribution()->setRandomEngine(randomEngine);
	RobotStateSharedPtr opptState = static_cast<DespotState &>(state).getOpptState();

	// Make the next state
	PropagationRequestSharedPtr propagationRequest(new PropagationRequest);
	propagationRequest->currentState = opptState;
	propagationRequest->action = actions_[action];
	propagationRequest->allowCollisions = options->allowCollisions;
	PropagationResultSharedPtr propagationResult =
	    robot->propagateState(propagationRequest);
	static_cast<DespotState &>(state).setOpptState(propagationResult->nextState);

	// Sample an observation
	ObservationRequestSharedPtr observationRequest(new ObservationRequest);
	observationRequest->currentState = propagationResult->nextState;
	observationRequest->action = propagationRequest->action;
	ObservationResultSharedPtr observationResult =
	    robot->makeObservationReport(observationRequest);

	auto res = observationMap_->emplace(observationResult->observation, observationMap_->getMap()->size());
	// Get the reward
	reward = robotEnvironment->getReward(propagationResult);
	bool terminal = robotEnvironment->isTerminal(propagationResult);
	return terminal;
}

bool DespotModel::Step(despot::State& state, despot::ACT_TYPE action, double& reward,
                       despot::OBS_TYPE& obs) const {
	auto options = static_cast<const ABTExtendedOptions *>(problemEnvironment_->getOptions());
	auto robotEnvironment = problemEnvironment_->getRobotPlanningEnvironment();
	auto robot = problemEnvironment_->getRobotPlanningEnvironment()->getRobot();
	RobotStateSharedPtr opptState = static_cast<DespotState &>(state).getOpptState();

	// Make the next state
	PropagationRequestSharedPtr propagationRequest(new PropagationRequest);
	propagationRequest->currentState = opptState;
	propagationRequest->action = actions_[action];
	propagationRequest->allowCollisions = options->allowCollisions;
	PropagationResultSharedPtr propagationResult =
	    robot->propagateState(propagationRequest);
	static_cast<DespotState &>(state).setOpptState(propagationResult->nextState);

	// Sample an observation
	ObservationRequestSharedPtr observationRequest(new ObservationRequest);
	observationRequest->currentState = propagationResult->nextState;
	observationRequest->action = propagationRequest->action;
	ObservationResultSharedPtr observationResult =
	    robot->makeObservationReport(observationRequest);

	auto res = observationMap_->emplace(observationResult->observation, observationMap_->getMap()->size());
	obs = res.first->second;	

	// Get the reward
	reward = robotEnvironment->getReward(propagationResult);
	return robotEnvironment->isTerminal(propagationResult);
}

int DespotModel::NumActions() const {
	return actions_.size();
}

double DespotModel::Reward(const despot::State& state, despot::ACT_TYPE action) const {
	PropagationResultSharedPtr propRes(new PropagationResult);
	propRes->previousState = static_cast<const DespotState &>(state).getPreviousState();
	propRes->nextState = static_cast<const DespotState &>(state).getOpptState();
	propRes->action = actions_[action].get();
	return problemEnvironment_->getRobotPlanningEnvironment()->getReward(propRes);
}

double DespotModel::ObsProb(despot::OBS_TYPE obs, const despot::State& state,
                            despot::ACT_TYPE action) const {
	return 1.0;
}

despot::State* DespotModel::CreateStartState(std::string type) const {
	ERROR("Implement CreateStartState");
}

despot::Belief* DespotModel::InitialBelief(const despot::State* start,
        std::string type) const {
	unsigned int numParticles =
	    static_cast<const ABTExtendedOptions *>(problemEnvironment_->getOptions())->minParticleCount;
	unsigned int numScenarios =
	    static_cast<const ABTExtendedOptions *>(problemEnvironment_->getOptions())->numScenarios;
	std::vector<despot::State *> particles(numParticles, nullptr);
	for (size_t i = 0; i != particles.size(); ++i) {
		RobotStateSharedPtr opptState = problemEnvironment_->getRobotPlanningEnvironment()->sampleInitialState();
		despot::State *despotState = Allocate(i, 1.0 / numScenarios);
		static_cast<DespotState *>(despotState)->setOpptState(opptState);
		particles[i] = despotState;
	}

	despot::Belief *initBelief = new Belief(this, particles);

	static_cast<Belief *>(initBelief)->setObservationMap(observationMap_.get());
	static_cast<Belief *>(initBelief)->setActions(actions_);
	static_cast<Belief *>(initBelief)->robotEnvironment_ = problemEnvironment_->getRobotPlanningEnvironment();
	belief_ = initBelief;
	return initBelief;
}

despot::Belief *DespotModel::getBelief() const {
	return belief_;
}

double DespotModel::GetMaxReward() const {
	cout << "ub" << endl;
	return problemEnvironment_->getRobotPlanningEnvironment()->getRewardPlugin()->getMinMaxReward().second;
}

despot::ValuedAction DespotModel::GetBestAction() const {
	despot::ValuedAction valuedAction;
	valuedAction.action = 0;
	valuedAction.value = problemEnvironment_->getRobotPlanningEnvironment()->getRewardPlugin()->getMinMaxReward().first;
	return valuedAction;
}

void DespotModel::PrintState(const despot::State& state, std::ostream& out) const {
	static_cast<const oppt::DespotState *>(&state)->print(out);
}

void DespotModel::PrintObs(const despot::State& state, despot::OBS_TYPE obs,
                           std::ostream& out) const {
	auto obsPair = observationMap_->find(obs);
	if (!(obsPair.first))
		ERROR("Observation not found");
	cout << *(obsPair.first.get()) << endl;
}

void DespotModel::PrintAction(despot::ACT_TYPE action, std::ostream& out) const {
	out << *(actions_[action].get()) << endl;
}

void DespotModel::PrintBelief(const despot::Belief& belief,
                              std::ostream& out) const {
	ERROR("Implement PrintBelief");
}


despot::State* DespotModel::Allocate(int state_id, double weight) const {
	DespotState* state = memoryPool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}


despot::State* DespotModel::Copy(const despot::State* state) const {
	DespotState* copied = memoryPool_.Allocate();
	*copied = *static_cast<const DespotState*>(state);

	copied->SetAllocated();
	return copied;
}


void DespotModel::Free(despot::State* state) const {
	memoryPool_.Free(static_cast<DespotState*>(state));
}


int DespotModel::NumActiveParticles() const {
	int numAllocated = memoryPool_.num_allocated();
	return numAllocated;
}

despot::ScenarioLowerBound* DespotModel::CreateScenarioLowerBound(std::string name,
        std::string particle_bound_name) const {
	despot::ScenarioLowerBound *lowerBound = new ParticleLowerBound(this);
	return lowerBound;

}

despot::ScenarioUpperBound* DespotModel::CreateScenarioUpperBound(std::string name,
        std::string particle_bound_name) const {
	despot::ScenarioUpperBound *upperBound = new ParticleUpperBound(this);
	return upperBound;
}

std::vector<ActionSharedPtr> DespotModel::getActions() const {
	return actions_;
}

/***************** POMCP Related functions ************************/
despot::POMCPPrior* DespotModel::CreatePOMCPPrior(std::string name) const {
	despot::POMCPPrior *prior = new OPPTPOMCPPrior(this);
	return prior;
}


}