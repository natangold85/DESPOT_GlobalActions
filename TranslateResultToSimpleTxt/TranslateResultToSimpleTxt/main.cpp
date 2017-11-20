#include <string>
#include <iostream>
#include <fstream>

std::string resultFname("out.txt");

void ErrorAndExit(const char *func)
{
	
	std::cout << "error in " << func << "\npress any key to exit\n";
	char c;
	std::cin >> c;
}

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

	char buffer[1024];
	txtFile.read(buffer, 1023);
	if (txtFile.bad())
		ErrorAndExit("read");

	buffer[1023] = 0;
	std::string txt;
	txtFile >> txt;
	if (txtFile.bad())
		ErrorAndExit("read");

	std::ofstream cleanFile(cleanResults.c_str(), std::ios::out);
	
	char *endPtr;
	int numRuns = strtod(txt.c_str(), &endPtr);


	return 0;
}