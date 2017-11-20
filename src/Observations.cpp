#include "Observations.h"
#include "Coordinate.h"

#include <string>

// constant for 
static double s_SQRT2 = sqrt(2);

ObservationByDistance::ObservationByDistance(double distanceFactor, double nonObserved)
: m_distanceFactor(distanceFactor)
, m_nonObserved(nonObserved)
, m_observationDivergence(1 - nonObserved)
{
}

bool ObservationByDistance::InRange(int selfLoc, int obsObjLoc, int gridSize) const
{
	// observed range is in all grid
	return true;
}

double ObservationByDistance::GetProbObservation(int selfLoc, int objLoc, int gridSize, int observation) const
{
	// TODO: implement probability of divergence

	int nonObsLoc = selfLoc;
	// only 2 possibilities for valid observation(non observed = self location)
	if ((observation != objLoc) & (observation != nonObsLoc))
		return 0.0;

	// if object is dead we have 1.0 prob to observe it
	if (objLoc == gridSize * gridSize)
		return 1.0 * (observation == gridSize * gridSize);

	Coordinate self(selfLoc % gridSize, selfLoc / gridSize);
	Coordinate obj(objLoc % gridSize, objLoc / gridSize);

	// put weight on closer targets
	double dist = self.RealDistance(obj);
	double ratio = dist / (s_SQRT2 * gridSize);

	// calculate prob to success observation
	double pSuccess = ratio * m_distanceFactor + (1 - ratio);

	return pSuccess * (observation == objLoc) + (1 - pSuccess) * (observation == nonObsLoc);
}

int ObservationByDistance::GetObservationObject(int selfLoc, int objLoc, int gridSize, double randomNum) const
{
	if (objLoc = gridSize * gridSize)
		return objLoc;

	Coordinate self(selfLoc % gridSize, selfLoc / gridSize);
	Coordinate obj(objLoc % gridSize, objLoc / gridSize);

	double dist = self.RealDistance(obj);
	double ratio = dist / (s_SQRT2 * gridSize);

	// calculate prob to success observation
	double pSuccess = ratio * m_distanceFactor + (1 - ratio);

	randomNum -= pSuccess;
	if (randomNum < 0)
		return objLoc;

	randomNum -= m_nonObserved * (1 - pSuccess);
	if (randomNum < 0)
		return selfLoc;

	// TODO : implement divergence observations
}

void ObservationByDistance::InitObsAvailableLocations(int selfLoc, int objLoc, const intVec & state, int gridSize, intVec & observableLocations) const
{
	//TODO: until divergence insertion only 2 possible location (observed and non-observed)
	observableLocations.emplace_back(objLoc);

	// insert non-observed location if the object is not dead
	if (objLoc != gridSize * gridSize)
		observableLocations.emplace_back(selfLoc);
}

std::string ObservationByDistance::String() const
{
	std::string ret = "Observation by distance: distance factor = ";
	ret += std::to_string(m_distanceFactor) + ", rest prob: non observed = " + std::to_string(m_nonObserved) + " divergence = " + std::to_string(m_observationDivergence);
	return ret;
}
