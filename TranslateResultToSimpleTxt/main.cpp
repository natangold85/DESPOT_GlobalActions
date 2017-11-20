#include <string>
#include <iostream>
#include <fstream>

std::string resultFname("naive_result.txt");

void ErrorAndExit(const char *func)
{
	std::cout << "error in " << func << "\npress any key to exit\n";
	char c;
	std::cin >> c;
	exit(1);
}

void WriteNumSteps(std::ofstream & out, std::ifstream & in, int numRounds);
void WriteRewards(std::ofstream & out, std::ifstream & in, int numRounds);
void WriteHeight(std::ofstream & out, std::ifstream & in, int numRounds);
void WriteSize(std::ofstream & out, std::ifstream & in, int numRounds);
void WriteLevelSizes(std::ofstream & out, std::ifstream & in, int numRounds);
bool IsNextLevelAction(std::ifstream & in);
void WriteLevelActionSizes(std::ofstream & out, std::ifstream & in, int numRounds);
void WriteLevelPortion(std::ofstream & out, std::ifstream & in, int numRounds);

int main()
{
	std::ifstream txtFile(resultFname.c_str(), std::ios::in);
	if (txtFile.fail())
		ErrorAndExit("open");

	std::string cleanResults(resultFname);
	cleanResults.pop_back();
	cleanResults.pop_back();
	cleanResults.pop_back();
	cleanResults.pop_back();
	cleanResults += "_Clean.txt";

	std::string txt;
	while (txt != "=")
		txtFile >> txt;

	int junk;

	int numRuns;
	txtFile >> numRuns;

	// read number of rounds for each run
	int numRounds;
	while (txt != "Completed")
		txtFile >> txt;
	txtFile >> numRounds;

	if (txtFile.bad())
		ErrorAndExit("read");

	std::ofstream cleanFile(cleanResults.c_str(), std::ios::out);

	for (int i = 0; i < numRuns; ++i)
	{
		WriteNumSteps(cleanFile, txtFile, numRounds);
		WriteRewards(cleanFile, txtFile, numRounds);
		WriteHeight(cleanFile, txtFile, numRounds);
		WriteSize(cleanFile, txtFile, numRounds);
		WriteLevelSizes(cleanFile, txtFile, numRounds);
		WriteLevelPortion(cleanFile, txtFile, numRounds);

		cleanFile << "\n\n";
	}

	cleanFile.close();
	return 0;
}

void WriteNumSteps(std::ofstream & out, std::ifstream & in, int numRounds)
{
	std::string txt;
	while (txt != "run:")
		in >> txt;

	for (int i = 0; i < numRounds; ++i)
	{
		in >> txt;
		out << txt;
	}

	out << "\n";
}

void WriteRewards(std::ofstream & out, std::ifstream & in, int numRounds)
{
	std::string txt;
	while (txt != "reward:")
		in >> txt;

	for (int i = 0; i < numRounds; ++i)
	{
		in >> txt;
		out << txt;
	}

	out << "\n";
}

void WriteHeight(std::ofstream & out, std::ifstream & in, int numRounds)
{
	std::string txt;
	while (txt != "Height:")
		in >> txt;

	for (int i = 0; i < numRounds; ++i)
	{
		in >> txt;
		out << txt;
	}
	out << "\n";
}

void WriteSize(std::ofstream & out, std::ifstream & in, int numRounds)
{
	std::string txt;
	while (txt != "Size:")
		in >> txt;

	for (int i = 0; i < numRounds; ++i)
	{
		in >> txt;
		out << txt;
	}
	out << "\n";
}

void WriteLevelSizes(std::ofstream & out, std::ifstream & in, int numRounds)
{
	std::string txt;
	while (txt != "sizes:")
		in >> txt;

	while (txt != "level" & !in.eof())
	{
		for (int i = 0; i < numRounds; ++i)
		{
			in >> txt;
			
			if (txt == "level")
				break;

			out << txt;
		}
		out << "\n";
	}
}

bool IsNextLevelAction(std::ifstream & in)
{
	std::string txt;
	while (txt != "sizes:" & txt != "run" & !in.eof())
		in >> txt;

	return txt == "sizes:";
}

void WriteLevelActionSizes(std::ofstream & out, std::ifstream & in, int numRounds)
{
	std::string txt;

	while (txt != "level")
	{
		for (int i = 0; i < numRounds; ++i)
		{
			in >> txt;
			if (txt == "level")
				break;
			out << txt;
		}
		out << "\n";
	}
}
void WriteLevelPortion(std::ofstream & out, std::ifstream & in, int numRounds)
{
	std::string txt;
	while (txt != "portion:")
		in >> txt;

	while (txt != "run" & !in.eof())
	{
		for (int i = 0; i < numRounds; ++i)
		{
			in >> txt;
			if (txt == "run" | in.eof())
				break;
			out << txt;
		}
		out << "\n";
	}
}