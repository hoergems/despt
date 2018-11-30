#ifndef _POMCP_PRIOR_HPP_
#define _POMCP_PRIOR_HPP_
#include <despot/solver/pomcp.h>
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>

namespace oppt {

class OPPTPOMCPPrior: public despot::POMCPPrior {
public:
	OPPTPOMCPPrior(const despot::DSPOMDP* model);

	virtual ~OPPTPOMCPPrior() = default;

	virtual void ComputePreference(const despot::State& state) override;

	inline virtual void exploration_constant(double constant) override {

	}

	inline virtual double exploration_constant() const override {
		return exploration_constant_;
	}

	virtual despot::RolloutStrategy getRolloutStrategy() const override;

private:
	ProblemEnvironment *problemEnvironment_ = nullptr;

	despot::RolloutStrategy heuristicRollout_ = nullptr;
};

}

#endif