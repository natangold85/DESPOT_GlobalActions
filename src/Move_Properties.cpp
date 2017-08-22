#include "Move_Properties.h"

Move_Properties::Move_Properties()
	: m_pEqualShare(0.0)
	, m_pStay(0.0)
	, m_pTowardTarget(0.0)
{
}

Move_Properties::Move_Properties(double stay, double towardTarget)
	: m_pEqualShare( (1 - stay - towardTarget) / s_numDirections )
	, m_pStay(stay)
	, m_pTowardTarget(towardTarget)
{
}

std::ofstream & operator<<(std::ofstream & out, const Move_Properties & obj)
{
	out << obj.m_pEqualShare << obj.m_pStay << obj.m_pTowardTarget;
	return out;
}

std::ifstream & operator>>(std::ifstream & in, Move_Properties & obj)
{
	in >> obj.m_pEqualShare >> obj.m_pStay >> obj.m_pTowardTarget;

	return in;
}
