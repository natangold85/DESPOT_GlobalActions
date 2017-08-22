#include "Attack_Obj.h"



Attack_Obj::Attack_Obj(Point& location, Move_Properties& movement, double attackRange, double pHit)
: Movable_Obj(location, movement)
, m_attack( new DirectAttack(attackRange, pHit) )
{
}

std::ofstream & operator<<(std::ofstream & out, const Attack_Obj & obj)
{
	out.write(reinterpret_cast<const char *>(&obj), sizeof(Attack_Obj));
	return out;
}

std::ifstream & operator>>(std::ifstream & in, Attack_Obj & obj)
{
	in.read(reinterpret_cast<char *>(&obj), sizeof(Attack_Obj));
	return in;
}

