#include "Self_Obj.h"



Self_Obj::Self_Obj(Point& location, Move_Properties& movement, double attackRange, double pHit, int rangeObs, double pObservation)
: Attack_Obj(location, movement, attackRange, pHit)
, m_rangeObs(rangeObs)
, m_pObservation(pObservation)
{
}

std::ofstream & operator<<(std::ofstream & out, const Self_Obj & obj)
{
	//const Attack_Obj base = static_cast<const Attack_Obj >(obj);
	//out << base;

	//out.write(reinterpret_cast<const char *>(&obj.m_rangeObs), sizeof(int));
	//out.write(reinterpret_cast<const char *>(&obj.m_rangeObs), sizeof(double));
	out.write(reinterpret_cast<const char *>(&obj), sizeof(Self_Obj));
	return out;
}

std::ifstream & operator>>(std::ifstream & in, Self_Obj & obj)
{
	in.read(reinterpret_cast<char *>(&obj), sizeof(Self_Obj));
	return in;
}
