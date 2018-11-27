#ifndef _DESPOT_BOUND_HPP_
#define _DESPOT_BOUND_HPP_
#include <despot/interface/pomdp.h>
#include <oppt/opptCore/core.hpp>

namespace oppt {

class ParticleLowerBound: public despot::ParticleLowerBound {
public:
	ParticleLowerBound(const despot::DSPOMDP* model);

	virtual ~ParticleLowerBound();

	virtual despot::ValuedAction Value(const std::vector<despot::State*>& particles) const override;

private:
	const despot::DSPOMDP *model_ = nullptr;

};

class ParticleUpperBound: public despot::ParticleUpperBound {
public:
	ParticleUpperBound(const despot::DSPOMDP* model);

	virtual ~ParticleUpperBound();

	virtual double Value(const despot::State& state) const override;

private:
	const despot::DSPOMDP *model_ = nullptr;
};

/************* Scenario Bound *******************/
class ScenarioLowerBound: public despot::ScenarioLowerBound {
public:
	ScenarioLowerBound(const despot::DSPOMDP* model);

	virtual ~ScenarioLowerBound();

	virtual despot::ValuedAction Value(const std::vector<despot::State*>& particles,
		despot::RandomStreams& streams, despot::History& history) const override;

};

}

#endif