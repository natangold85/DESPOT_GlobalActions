#include "Movable_Obj.h"


Movable_Obj::Movable_Obj(Coordinate& location, Move_Properties& movement)
	: ObjInGrid(location)
	, m_movement(movement)
{
}

std::ofstream & operator<<(std::ofstream & out, const Movable_Obj & obj)
{
	out.write(reinterpret_cast<const char *>(&obj), sizeof(Movable_Obj));
	return out;
}

std::ifstream & operator>>(std::ifstream & in, Movable_Obj & obj)
{
	in.read(reinterpret_cast<char *>(&obj), sizeof(Movable_Obj));
	return in;
}