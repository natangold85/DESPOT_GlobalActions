#ifndef MOVABLE_OBJ_H
#define MOVABLE_OBJ_H

#include <fstream>      // std::ofstream

#include "ObjInGrid.h"
#include "Move_Properties.h"
#include "Coordinate.h"

///class of movable objects in grid for pomdp writer. can be use as non-involved objects
class Movable_Obj :
	public ObjInGrid
{
public:
	explicit Movable_Obj() = default;
	explicit Movable_Obj(Coordinate& location, Move_Properties& movement);
	virtual ~Movable_Obj() = default;
	Movable_Obj(const Movable_Obj &) = default;
	Movable_Obj& operator=(const Movable_Obj&) = default;

	///return object movement properties
	const Move_Properties& GetMovement() const { return m_movement; }
	///change object movement prop
	void SetMovement(Move_Properties m) { m_movement = m; }

	/// write object to file
	friend std::ofstream& operator<<(std::ofstream& out, const Movable_Obj& obj);
	/// read object from file
	friend std::ifstream& operator>>(std::ifstream& in, Movable_Obj& obj);

private:
	Move_Properties m_movement;
};

# endif //MOVABLE_OBJ_H