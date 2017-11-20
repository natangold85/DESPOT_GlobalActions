#ifndef NXNGRIDOFFLINEGLOBALACTIONS_H
#define NXNGRIDOFFLINEGLOBALACTIONS_H

#include "nxnGridOffline.h"

#include <string>

class nxnGridOfflineGlobalActions : public nxnGridOffline
{

public:
	explicit nxnGridOfflineGlobalActions() = default;
	explicit nxnGridOfflineGlobalActions(int gridSize, int targetIdx, Self_Obj& self, bool isFullyObs = false, double discount = 0.95);
	nxnGridOfflineGlobalActions(const nxnGridOfflineGlobalActions &) = default;
	nxnGridOfflineGlobalActions& operator=(const nxnGridOfflineGlobalActions&) = default;

	enum BASIC_ACTION { MOVE_TO_TARGET, MOVE_TO_SHELTER, NUM_BASIC_ACTIONS }; 
	enum ENEMY_ACTION { ATTACK, MOVE_FROM_ENEMY, NUM_ENEMY_ACTIONS };

	/// save model in pomdp format to file
	void SaveInPomdpFormat(FILE *fptr) override;

	virtual int GetNumActions() const override;
private:
	// main functions for saving format to file: 

	/// insert init lines, init states, state list and comments to file
	void CommentsAndInitLines(FILE *fptr) override;
	/// insert transitions of all actions to file
	void AddAllActions(FILE *fptr) override;

	std::string AddActionsString();

	// Calculation of actions result
	void AddActionsAllStates(std::string & buffer);
	/// run on all possible states and calculate the end-state fro all actions
	void AddActionsRec(intVec & state, intVec & shelters, int currObj, std::string & buffer);

	/// add action attack with state and shelters to buffer
	void AddAttack(intVec & state, int enemyIdx, intVec & shelters, std::string & buffer) const;
	/// add action move to target with state and shelters to buffer
	void AddMoveToTarget(intVec & state, intVec & shelters, std::string & buffer) const;
	/// add action move to shelter with state and shelters to buffer
	void AddMoveToShelter(intVec & state, intVec & shelters, std::string & buffer) const;
	/// add action move from enemy with state and shelters to buffer
	void AddMoveFromEnemy(intVec & state, int idxEnemy, intVec & shelters, std::string & buffer) const;

	/// move to a specific location return the peobability to loss in that action
	double MoveToLocation(intVec & state, intVec & shelters, int location, std::string & action, std::string & buffer) const;
	/// move to location return new self location (-1 if there is no way to get closer to location)
	int MoveToLocationIMP(intVec & state, int goTo) const;
	/// return the farthest point reachable of self from a specific location
	int MoveFromLocation(intVec & state, int location) const;
};

# endif //NXNGRIDOFFLINEGLOBALACTIONS_H