#ifndef SELF_OBJ_H
#define SELF_OBJ_H

#include <vector>      // vector

#include "Attack_Obj.h"
#include "Coordinate.h"
#include "Observations.h"

/// class for self object in nxnGrid
class Self_Obj : public Attack_Obj
{
public:
	using intVec = std::vector<int>;

	explicit Self_Obj() = default;
	explicit Self_Obj(Coordinate& location, Move_Properties& movement, std::shared_ptr<Attack> attack, std::shared_ptr<Observation> obs);
	virtual ~Self_Obj() = default;
	Self_Obj(const Self_Obj&) = default;

	double GetSelfPStay() const { return GetMovement().GetStay(); };
	double GetSelfPMove() const { return GetMovement().GetToward(); };

	/// specific implementation for self objects because for now only self object can kill non involved

	/// calculation of attack for online solver given locations, gridSize and random number 
	virtual void AttackOnline(int objLoc, int targetLoc, intVec & state, intVec & shelters, int gridSize, double random) const override;
	/// calculation of attack for offline solver given locations, gridSize. attack result are update in result 
	virtual void AttackOffline(int objLoc, int targetLoc, intVec & state, intVec & shelters, int gridSize, Attack::shootOutcomes & result) const;

	Observation *GetObservation() const  { return m_observation.get(); };

private:
	std::shared_ptr<Observation> m_observation;
};

# endif //SELF_OBJ_H