#include <fstream>
#include <vector>

int s_gridSize = 5;
int s_numObj = 4;
static std::string s_lutName("5x5Grid2x0x1_LUT.bin");
static std::string s_outFName("5x5Grid2x0x1_LUT.txt");


void Idx2State(int idx, int gridSize, int numObj, std::vector<int> & state);

int main()
{
	std::ifstream lut(s_lutName, std::ios::in | std::ios::binary);

	std::ofstream lutTxt(s_outFName, std::ios::out);

	int size;
	lut.read((char *)&size, sizeof(int));
	int numActions;
	lut.read((char *)&numActions, sizeof(int));

	for (int i = 0; i < size; ++i)
	{
		int stateIdx;
		lut.read((char *)&stateIdx, sizeof(int));

		std::vector<int> state;
		Idx2State(stateIdx, s_gridSize, s_numObj, state);

		lutTxt << "\n\nfor state (";
		for (auto v : state)
			lutTxt << v << ", ";
		lutTxt << ")\n";
		for (int a = 0; a < numActions; ++a)
		{
			double reward;
			lut.read((char *)&reward, sizeof(double));
			lutTxt << "action = " << a << " reward = " << reward << ", ";
		}
	}
	return 0;
}


void Idx2State(int idx, int gridSize, int numObj, std::vector<int> & state)
{
	state.resize(numObj);
	int numStates = gridSize * gridSize + 1;

	// running on all varied objects and concluding from the obs num the observed state
	for (int i = numObj - 1; i >= 0; --i)
	{
		state[i] = idx % numStates;
		idx /= numStates;
	}
}

