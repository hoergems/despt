#ifndef _DESPOT_OBSERVATION_MAP_HPP_
#define _DESPOT_OBSERVATION_MAP_HPP_
#include <oppt/opptCore/core.hpp>

namespace oppt {

struct Hasher {
	std::size_t operator()(const ObservationSharedPtr& obs) const {
		return obs->hash();
	}
};

struct Equalizer {

	Equalizer(const double &maxDist):
		maxDist_(maxDist_) {

	}

	bool operator()(const ObservationSharedPtr& obsOne, const ObservationSharedPtr& obsTwo) const {
		double dist = obsOne->distanceTo(*(obsTwo.get())) <= maxDist_;
		if (dist <= maxDist_)
			return true;
		return false;
	}

	double maxDist_;
};

typedef std::unordered_map<const ObservationSharedPtr,
        size_t,
        Hasher,
        std::function<bool(const ObservationSharedPtr&, const ObservationSharedPtr&)>> ObservationIntMap;

typedef std::unique_ptr<ObservationIntMap> ObservationIntMapPtr;

class ObservationMap {
public:
	ObservationMap(const FloatType &maxObservationDistance):
		map_(0, Hasher(), [ & maxObservationDistance](const ObservationSharedPtr & o1, const ObservationSharedPtr & o2) {			
		if (o1->distanceTo(*(o2.get())) <= maxObservationDistance)
			return true;
		return false;
	}),
	maxObservationDistance_(maxObservationDistance) {

	}

	const ObservationIntMap *getMap() const {
		return &map_;
	}

	std::pair<ObservationIntMap::iterator, bool> insert(const ObservationSharedPtr &observation, const size_t &val) {
		return map_.insert({observation, val});
	}

	std::pair<ObservationIntMap::iterator, bool> emplace(const ObservationSharedPtr &observation,
	        const size_t &val, const bool &enforce = false) {
		if (!enforce) {
			for (auto it = map_.begin(); it != map_.end(); it++) {
				if (it->first->distanceTo(*(observation.get())) < maxObservationDistance_) {
					std::pair<ObservationIntMap::iterator, bool> p = {it, false};
					return p;
				}
			}
		}
		return map_.emplace(observation, val);
	}

	size_t size() const {
		return map_.size();
	}

	bool exists(const ObservationSharedPtr &observation) const {
		if (map_.find(observation) != map_.end())
			return true;
		return false;
	}

	size_t at(const ObservationSharedPtr &observation) const {
		return map_.at(observation);
	}

	std::pair<const ObservationSharedPtr &, size_t> find(const size_t &val) const {
		std::pair<const ObservationSharedPtr, size_t> p = {nullptr, val};
		for (auto it=map_.begin(); it != map_.end(); it++) {
			if (it->second == val) {
				std::pair<const ObservationSharedPtr, size_t> pFound = {it->first, val};
				return pFound;
			}
		}

		return p;
	}

private:
	ObservationIntMap map_;

	double maxObservationDistance_ = 0.0;

};

typedef std::unique_ptr<ObservationMap> ObservationMapPtr;
}

#endif