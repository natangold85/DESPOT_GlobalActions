#include "Sarsop_Writer.h"

size_t Sarsop_Writer::CountMovableObj() const
{
	return 1 + m_enemyVec.size() + m_NInvVector.size();
}

size_t Sarsop_Writer::CountEnemies() const
{
	return m_enemyVec.size();
}

size_t Sarsop_Writer::CountNInv() const
{
	return m_NInvVector.size();
}

size_t Sarsop_Writer::CountShelters() const
{
	return m_shelter.size();
}

size_t Sarsop_Writer::GetGridSize() const
{
	return m_gridSize;
}

bool operator<(const Sarsop_Writer & p1, const Sarsop_Writer & p2)
{
	return false;
}

std::ostream & operator<<(std::ostream & o, const Sarsop_Writer & map)
{
	// TODO: insert return statement here
}
