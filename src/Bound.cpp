#include "include/Bound.hpp"
#include "include/Model.hpp"
#include "include/State.hpp"
#include "include/OpptDummySolver.hpp"
#include <oppt/opptCore/core.hpp>
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>

namespace oppt {
ParticleLowerBound::ParticleLowerBound(const despot::DSPOMDP* model):
	despot::ParticleLowerBound(model),
	model_(model) {

}

ParticleLowerBound::~ParticleLowerBound() {

}

despot::ValuedAction ParticleLowerBound::Value(const std::vector<despot::State*>& particles) const {
	auto model = static_cast<const DespotModel *>(model_);
	auto actions = model->getActions();

	auto robotEnvironment = model->getProblemEnvironment()->getRobotPlanningEnvironment();
	auto heuristicPlugin = static_cast<solvers::OpptDummySolver *>(model->getProblemEnvironment()->getSolver())->getHeuristicPlugin();

	RandomEnginePtr randomEngine =
	    robotEnvironment->getRobot()->getRandomEngine();
	std::uniform_int_distribution<unsigned int> distr(0, particles.size() - 1);
	unsigned int randIdx = distr(*(randomEngine.get()));

	despot::State *randomState = particles[randIdx];
	RobotStateSharedPtr opptState = static_cast<DespotState *>(randomState)->getOpptState();

	PropagationRequestSharedPtr propagationRequest(new PropagationRequest);
	propagationRequest->currentState = opptState;
	std::unique_ptr<HeuristicInfo> heuristicInfo(new HeuristicInfo);
	FloatType lowerBound = std::numeric_limits<FloatType>::infinity();
	unsigned int bestAction = 0;
	std::vector<std::pair<unsigned int, FloatType>> goodActions;
	for (size_t i = 0; i != actions.size(); ++i) {
		propagationRequest->action = actions[i];
		PropagationResultSharedPtr propagationResult = robotEnvironment->getRobot()->propagateState(propagationRequest);
		bool terminal = robotEnvironment->isTerminal(propagationResult);
		FloatType val = -std::numeric_limits<FloatType>::infinity();
		if (terminal) {
			val = robotEnvironment->getReward(propagationResult);
			if (val < 0.0) {
				lowerBound = val;
				bestAction = i;
				break;
			}
		} else {
			heuristicInfo->currentState = propagationResult->nextState;
			val = 0.9 * heuristicPlugin->getHeuristicValue(heuristicInfo.get());			
		}	

		if (val < lowerBound) {
			lowerBound = val;
			bestAction = i;
		}
	}

	despot::ValuedAction valuedAction;
	valuedAction.action = bestAction;
	valuedAction.value = lowerBound;
	return valuedAction;
	/**despot::ValuedAction valuedAction;
	valuedAction.action = 0;
	valuedAction.value = -1;
	return valuedAction;*/

}


ParticleUpperBound::ParticleUpperBound(const despot::DSPOMDP* model):
	despot::ParticleUpperBound(),
	model_(model) {

}

ParticleUpperBound::~ParticleUpperBound() {

}

double ParticleUpperBound::Value(const despot::State& state) const {
	auto model = static_cast<const DespotModel *>(model_);
	auto robotEnvironment = model->getProblemEnvironment()->getRobotPlanningEnvironment();
	FloatType ub = (FloatType)(robotEnvironment->getRewardPlugin()->getMinMaxReward().second);
	//return ub;
	RobotStateSharedPtr opptState = static_cast<const DespotState&>(state).getOpptState();
	std::unique_ptr<HeuristicInfo> heuristicInfo(new HeuristicInfo);
	heuristicInfo->currentState = opptState;
	auto heuristicPlugin = static_cast<solvers::OpptDummySolver *>(model->getProblemEnvironment()->getSolver())->getHeuristicPlugin();
	FloatType upperBound = heuristicPlugin->getHeuristicValue(heuristicInfo.get());	
	return upperBound;
	return ub;
	
	auto actions = model->getActions();	
	

	PropagationRequestSharedPtr propagationRequest(new PropagationRequest);
	propagationRequest->currentState = opptState;
	VectorFloat immediateRewards;
	for (size_t i = 0; i != actions.size(); ++i) {
		propagationRequest->action = actions[i];
		PropagationResultSharedPtr propagationResult = robotEnvironment->getRobot()->propagateState(propagationRequest);
		FloatType rew = robotEnvironment->getReward(propagationResult);
		if (rew < -100.0)
			immediateRewards.push_back(rew);
	}

	if (immediateRewards.size() == actions.size()) {
		WARNING("WOA");
		cout << immediateRewards[0] << endl;
		return immediateRewards[0];
	}




	return robotEnvironment->getRewardPlugin()->getMinMaxReward().second;
	ERROR("Implement ParticleUpperBound::Value");
}

/********************ScenarionLowerBound *****************/
ScenarioLowerBound::ScenarioLowerBound(const despot::DSPOMDP* model):
	despot::ScenarioLowerBound(model) {

}

ScenarioLowerBound::~ScenarioLowerBound() {

}

despot::ValuedAction ScenarioLowerBound::Value(const std::vector<despot::State*>& particles,
        despot::RandomStreams& streams, despot::History& history) const {

}

}