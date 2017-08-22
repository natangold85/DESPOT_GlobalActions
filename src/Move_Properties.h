#pragma once
#include <fstream>      // std::ofstream

///properties of movement for object on grid
class Move_Properties
{
public:
	explicit Move_Properties();
	explicit Move_Properties(double stay, double towardTarget = 0.0);
	~Move_Properties() = default;
	Move_Properties(const Move_Properties &) = default;
	Move_Properties& operator=(const Move_Properties&) = default;

	/// return the probability to move to each direction
	double GetEqual() const {return m_pEqualShare;}
	/// return the probability to stay in place
	double GetStay() const { return m_pStay; }
	/// return the probability to advance toward the target
	double GetToward() const { return m_pTowardTarget; }

	/// write obj to file
	friend std::ofstream& operator<<(std::ofstream& out, const Move_Properties& obj);
	/// read obj from file
	friend std::ifstream& operator>>(std::ifstream& in, Move_Properties& obj);

private:
	double m_pEqualShare;
	double m_pStay;
	double m_pTowardTarget;

	static const int s_numDirections = 8;
};

