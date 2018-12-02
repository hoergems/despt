#ifndef _DESPOT_MODEL_HPP_
#define _DESPOT_MODEL_HPP_
#include <despot/interface/pomdp.h>
#include <oppt/opptCore/core.hpp>
#include "ObservationMap.hpp"
#include "include/Belief.hpp"

namespace oppt {
class ProblemEnvironment;

class DespotState;

class DespotModel: public despot::DSPOMDP {
public:
	DespotModel();

	virtual ~DespotModel();

	void setProblemEnvironment(oppt::ProblemEnvironment *problemEnvironment);

	oppt::ProblemEnvironment *getProblemEnvironment() const;

	virtual bool Step(despot::State& state, double random_num, despot::ACT_TYPE action,
	                  double& reward, despot::OBS_TYPE& obs) const override;

	virtual bool Step(despot::State& state, despot::ACT_TYPE action, double& reward,
		despot::OBS_TYPE& obs) const override;

	virtual int NumActions() const override;

	virtual double Reward(const despot::State& state, despot::ACT_TYPE action) const override;

	virtual double ObsProb(despot::OBS_TYPE obs, const despot::State& state,
	                       despot::ACT_TYPE action) const override;

	virtual despot::State* CreateStartState(std::string type = "DEFAULT") const override;

	virtual despot::Belief* InitialBelief(const despot::State* start,
	                                      std::string type = "DEFAULT") const override;

	virtual double GetMaxReward() const override;

	virtual despot::ValuedAction GetBestAction() const override;

	virtual void PrintState(const despot::State& state, std::ostream& out = std::cout) const override;

	virtual void PrintObs(const despot::State& state, despot::OBS_TYPE obs,
	                      std::ostream& out = std::cout) const override;

	virtual void PrintAction(despot::ACT_TYPE action, std::ostream& out = std::cout) const override;

	virtual void PrintBelief(const despot::Belief& belief,
	                         std::ostream& out = std::cout) const override;

	virtual despot::POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const override;

	/* ========================================================================
	 * Memory management.
	 * ========================================================================*/
	/**
	 * [Essential]
	 * Allocate a state.
	 * @param state_id ID of the allocated state in the state space
	 * @param weight   Weight of the allocated state
	 */
	virtual despot::State* Allocate(int state_id = -1, double weight = 0) const override;

	/**
	 * [Essential]
	 * Returns a copy of the state.
	 * @param state The state to be copied
	 */
	virtual despot::State* Copy(const despot::State* state) const override;

	/**
	 * [Essential]
	 * Free the memory of a state.
	 * @param state The state to be freed
	 */
	virtual void Free(despot::State* state) const override;

	/**
	 * [Essential]
	 * Returns number of allocated particles (sampled states).
	 */
	virtual int NumActiveParticles() const override;

	/**
	 * [Optional]
	 * Returns a copy of this POMDP model.
	 */
	inline virtual despot::DSPOMDP* MakeCopy() const override {
		return NULL;
	}

	virtual despot::ScenarioLowerBound* CreateScenarioLowerBound(std::string bound_name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const override;

	virtual despot::ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const override;

	std::vector<ActionSharedPtr> getActions() const;

	void setObservationMap(ObservationMapPtr observationMap);

	ObservationMap *getObservationMap() const;

	despot::Belief *getBelief() const;

private:
	ProblemEnvironment *problemEnvironment_ = nullptr;

	mutable despot::MemoryPool<DespotState> memoryPool_;

	std::vector<ActionSharedPtr> actions_;

	ObservationMapPtr observationMap_ = nullptr;

	mutable despot::Belief *belief_ = nullptr;

	mutable size_t stepCounter_ = 0;

private:
	void setDiscretizedActions();

};
}

#endif
