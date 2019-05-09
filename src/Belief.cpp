#include "include/State.hpp"
#include "include/Model.hpp"
#include <oppt/opptCore/core.hpp>
#include <oppt/problemEnvironment/ProblemEnvironment.hpp>
#include "ABTOptions.hpp"

using std::cout;
using std::endl;

namespace oppt {

Belief::Belief(const despot::DSPOMDP* model, std::vector<despot::State *> particles):
	despot::Belief(model),
	robotEnvironment_(nullptr),
	particles_(particles),
	particleFilter_(std::make_unique<ParticleFilter>()) {

}

despot::Belief* Belief::MakeCopy() const {
	ERROR("Implement MakeCopy");
}

VectorRobotStatePtr Belief::getOpptParticles() const {
	VectorRobotStatePtr opptParticles(particles_.size(), nullptr);
	for (size_t i = 0; i != opptParticles.size(); ++i) {
		opptParticles[i] = static_cast<DespotState *>(particles_[i])->getOpptState();
	}

	return opptParticles;
}

std::vector<despot::State*> Belief::Sample(int num) const {
	std::vector<despot::State *> samples(num, nullptr);
	if (particles_.empty())
		ERROR("Belief has no particles");
	std::uniform_int_distribution<unsigned int> distribution(0, particles_.size() - 1);
	RandomEnginePtr randGen = robotEnvironment_->getRobot()->getRandomEngine();
	for (size_t i = 0; i != num; ++i) {
		unsigned int randIdx = distribution(*(randGen.get()));
		samples[i] = particles_[randIdx];
	}

	return samples;
}

void Belief::Update(despot::ACT_TYPE action, despot::OBS_TYPE obs) {
	if (!observationMap_)
		ERROR("No ObservationMap");
	if (actions_.empty())
		ERROR("No actions");
	ActionSharedPtr opptAction = actions_[action];
	ObservationSharedPtr observation = nullptr;
	auto map = observationMap_->getMap();
	for (auto obsEntry : *map) {
		if (obsEntry.second == obs) {
			observation = obsEntry.first;
			break;
		}
	}

	cout << "act: " << action << endl;
	cout << "obs: " << obs << endl;

	if (!observation)
		ERROR("No observation");

	auto options = static_cast<const DespotModel *>(model_)->getProblemEnvironment()->getOptions();
	FloatType numScenarios = (FloatType)(static_cast<const ABTExtendedOptions *>(options)->numScenarios);

	FilterRequestPtr filterRequest = std::make_unique<FilterRequest>();

	for (size_t i = 0; i != particles_.size(); ++i) {
		auto particle = static_cast<DespotState *>(particles_[i]);
		filterRequest->previousParticles.push_back(std::make_shared<Particle>(particle->getOpptState(),
		        particle->weight));
	}
	filterRequest->allowTerminalStates = false;
	filterRequest->allowZeroWeightParticles = false;
	filterRequest->numParticles = static_cast<const ABTExtendedOptions *>(options)->minParticleCount;
	filterRequest->robotEnvironment = robotEnvironment_;
	filterRequest->action = opptAction;
	filterRequest->observation = observation;
	filterRequest->allowCollisions = robotEnvironment_->getRobot()->getCollisionsAllowed();
	filterRequest->randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
	if (filterRequest->previousParticles.empty())
		ERROR("Previous particles are empty");

	FilterResultPtr filterResult = particleFilter_->filter(filterRequest);
	if (filterResult->particles.empty())
		return;

	for (auto &particle: particles_) {
		static_cast<const DespotModel *>(model_)->Free(particle);
	}

	particles_ = std::vector<despot::State *>(filterResult->particles.size(), nullptr);
	for (size_t i = 0; i != particles_.size(); ++i) {
		despot::State *st = static_cast<const DespotModel *>(model_)->Allocate();
		auto opptState = filterResult->particles[i]->getState();
		static_cast<DespotState *>(st)->setOpptState(opptState);
		static_cast<DespotState *>(st)->weight = filterResult->particles[i]->getWeight();
		//static_cast<DespotState *>(st)->weight = (1.0 / numScenarios);//filterResult->particles[i]->getWeight();
		particles_[i] = st;
	}

	static_cast<const DespotModel *>(model_)->cleanup();

}

void Belief::setObservationMap(ObservationMap *observationMap) {
	observationMap_ = observationMap;
}

void Belief::setActions(const std::vector<ActionSharedPtr> &actions) {
	actions_ = actions;
}

}