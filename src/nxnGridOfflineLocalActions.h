#ifndef NXNGRIDOFFLINELOCALACTIONS_H
#define NXNGRIDOFFLINELOCALACTIONS_H

#include "nxnGridOffline.h"

#include <string>

//TODO : update local model

class nxnGridOfflineLocalActions : public nxnGridOffline
{

public:
	explicit nxnGridOfflineLocalActions() = default;
	explicit nxnGridOfflineLocalActions(int gridSize, int targetIdx, Self_Obj& self, bool isFullyObs = false, double discount = 0.95);
	nxnGridOfflineLocalActions(const nxnGridOfflineLocalActions &) = default;
	nxnGridOfflineLocalActions& operator=(const nxnGridOfflineLocalActions&) = default;

	enum ACTION { MOVE_TO_TARGET, MOVE_TO_SHELTER, ATTACK, MOVE_FROM_ENEMY, NUM_BASIC_ACTIONS };

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
	void AddAttack(intVec & state, intVec & shelters, std::string & buffer) const;
	/// add action all moves with state and shelters to buffer
	void AddAllMoves(intVec & state, intVec & shelters, std::string & buffer) const;
	void AddSingleMove(intVec & state, intVec & shelters, std::string action, Coordinate advance, std::string & buffer) const;

	/// move to a specific location return the peobability to loss in that action
	double MoveToLocation(intVec & state, intVec & shelters, int location, std::string & action, std::string & buffer) const;
	/// move to location return new self location (-1 if there is no way to get closer to location)
	int MoveToLocationIMP(intVec & state, int goTo) const;
};

# endif //NXNGRIDOFFLINELOCALACTIONS_H