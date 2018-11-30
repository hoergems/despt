#include "include/POMCPPrior.hpp"
#include "include/Model.hpp"
#include "include/State.hpp"
#include "ABTOptions.hpp"
#include "include/OpptDummySolver.hpp"

namespace oppt {
OPPTPOMCPPrior::OPPTPOMCPPrior(const despot::DSPOMDP* model):
	despot::POMCPPrior(model),
	problemEnvironment_(static_cast<const DespotModel *>(model)->getProblemEnvironment()) {
	exploration_constant_ = static_cast<const ABTExtendedOptions *>(problemEnvironment_->getOptions())->explorationConstant;

	//auto heuristicPlugin = static_cast<solvers::OpptDummySolver *>(problemEnvironment_->getSolver())->getHeuristicPlugin();
	
	despot::RolloutStrategy heuristicRollout = [this](const despot::State * state, const despot::DSPOMDP * model) -> double {
		if (!state)
			ERROR("State is null");
		HeuristicInfo heuristicInfo;
		heuristicInfo.currentState = static_cast<const DespotState *>(state)->getOpptState();		
		return static_cast<solvers::OpptDummySolver *>(problemEnvironment_->getSolver())->getHeuristicPlugin()->getHeuristicValue(&heuristicInfo);		
	};

	heuristicRollout_ = heuristicRollout;

}

void OPPTPOMCPPrior::ComputePreference(const despot::State& state) {

}

despot::RolloutStrategy OPPTPOMCPPrior::getRolloutStrategy() const {
	return heuristicRollout_;
}
}