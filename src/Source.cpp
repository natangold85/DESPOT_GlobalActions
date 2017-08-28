#include <fstream>      // std::ofstream

/// tui class
#include "../include/despot/simple_tui.h"
#include "../include/despot/solver/pomcp.h"

/// models available
#include "nxnGrid.h"

/// for nxnGrid
#include "..\CreateLUT\StateActionLUT.h"
#include "MapOfPomdp.h"
#include "Coordinate.h"
#include "Move_properties.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "Self_Obj.h"
#include "ObjInGrid.h"

using namespace despot;

const char * BACKUPFILENAME = "trash";
const static char * LUTFILENAME = "5x5(E&S)_LUT(0.2).bin";

Attack_Obj CreateEnemy(int x, int y, int gridSize)
{
	int attackRange = gridSize / 4;
	attackRange += (attackRange == 0);

	double pHit = 0.5;
	double pStay = 0.4;
	double pTowardSelf = 0.4;

	Coordinate location(x,y);
	Move_Properties movement(pStay, pTowardSelf);
	return Attack_Obj(location, movement, attackRange, pHit);
}

Self_Obj CreateSelf(int x, int y, int gridSize)
{
	int attackRange = gridSize / 4;
	attackRange += (attackRange == 0);

	double pHit = 0.5;

	size_t observationRange = 2 * gridSize;
	double pSuccessObs = 0.9;

	double pMove = 0.9;
	double pStay = 1 - pMove;

	Coordinate location(x, y);
	Move_Properties movement(pStay, pMove);

	return Self_Obj(location, movement, attackRange, pHit, observationRange, pSuccessObs);
}

Movable_Obj CreateNInv(int x, int y)
{
	double pStay = 0.6;

	Coordinate location(x, y);
	Move_Properties movement(pStay);

	return Movable_Obj(location, movement);
}

ObjInGrid CreateShelter(int x, int y)
{
	double pStay = 0.6;

	Coordinate location(x, y);

	return ObjInGrid(location);
}

/// solve nxn Grid problem
class NXNGrid : public SimpleTUI {
public:
	NXNGrid() {}

	DSPOMDP* InitializeModel(option::Option* options) 
	{
		// create nxnGrid problem
		int gridSize = 10;

		Self_Obj self = CreateSelf(0, 0, gridSize);
		std::vector<intVec> key(1);
		key[0].resize(4);
		key[0][0] = 3;
		for (int i = 1; i < 4; ++i)
			key[0][i] = 1;

		std::shared_ptr<StateActionLUT> lut(new StateActionLUT(key, gridSize));
		
		std::ifstream readLut(LUTFILENAME, std::ios::in | std::ios::binary);
		if (readLut.fail())
		{
			std::cout << "failed open lut file for write\n\n\n";
			exit(1);
		}
		else
		{
			readLut >> *lut;
			if (readLut.bad())
			{
				std::cout << "failed write lut\n\n\n";
				exit(1);
			}
			else
				std::cout << "lut written succesfuly\n\n\n";

			readLut.close();
		}

		nxnGrid *model = new nxnGrid(gridSize, 0, self, lut, nxnGrid::RESCALE_ALL);

		model->AddObj(CreateEnemy(0, 0, gridSize));
		model->AddObj(CreateNInv(0, 0));
		model->AddObj(CreateShelter(0, 0));

		return model;
	}

	void InitializeDefaultParameters() {
	}
};

int main(int argc, char* argv[]) 
{
	srand(time(NULL));
		
	NXNGrid().run(argc, argv);
	return 0;
}
