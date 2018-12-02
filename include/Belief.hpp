#ifndef _DESPOT_OPPT_BELIEF_HPP_
#define _DESPOT_OPPT_BELIEF_HPP_
#include <despot/interface/pomdp.h>
#include <oppt/robotEnvironment/include/RobotEnvironment.hpp>
#include <oppt/filter/particleFilter/ParticleFilter.hpp>
#include "include/ObservationMap.hpp"

namespace oppt {

class DespotModel;

class Belief: public despot::Belief {
public:
	friend class DespotModel;
	Belief(const despot::DSPOMDP* model, std::vector<despot::State *> particles);

	virtual despot::Belief* MakeCopy() const override;

	virtual std::vector<despot::State*> Sample(int num) const override;
	virtual void Update(despot::ACT_TYPE action, despot::OBS_TYPE obs) override;

	void setObservationMap(ObservationMap *observationMap);

	void setActions(const std::vector<ActionSharedPtr> &actions);

	VectorRobotStatePtr getOpptParticles() const;

private:
	std::vector<despot::State*> particles_;

	const RobotEnvironment *robotEnvironment_ = nullptr;

	ObservationMap *observationMap_ = nullptr;

	std::vector<ActionSharedPtr> actions_;

	std::unique_ptr<oppt::ParticleFilter> particleFilter_ = nullptr;

};

}

#endif