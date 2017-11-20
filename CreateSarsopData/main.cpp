#include "RockSampleSarsop.h"

#include <fstream>
#include <iostream>
#include <Windows.h>	// CreateFile ..

// appl solver directory location
std::string s_SOLVER_LOCATION = "C:\\Users\\moshe\\Documents\\GitHub\\pomdp_solver\\src\\Release";

// buffer size for read
const static int s_BUFFER_SIZE = 1024;

static const double NON_VAL = -1000.0;

static HANDLE s_WRITE_PIPE, s_READ_PIPE;

static const std::string PomdpFName = "test.pomdp";
static const std::string policyFName = "test.policy";

/// init read and write pipes
void InitPipe(HANDLE *writePipe, HANDLE *readPipe)
{
	SECURITY_ATTRIBUTES sa;
	ZeroMemory(&sa, sizeof(sa));
	sa.nLength = sizeof(sa);
	sa.bInheritHandle = TRUE;

	if (!CreatePipe(readPipe, writePipe, &sa, 0))
	{
		std::cerr << "CreatePipe failed (" << GetLastError() << ")\n";
		exit(1);
	}
}

void RunSolver(const std::string &fname, const std::string &policy_name)
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	si.dwFlags |= STARTF_USESTDHANDLES;
	ZeroMemory(&pi, sizeof(pi));

	//run solver with timeout of 100 seconds and precision boundary of 0.005 (the first among these two conditions)
	std::string cmd = s_SOLVER_LOCATION + "\\pomdpsol.exe --timeout 100 ";
	cmd += fname + " -o " + policy_name;
	if (!CreateProcess(NULL, const_cast<char *>(cmd.c_str()), NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi))
	{
		std::cout << "CreateProcess failed (" << GetLastError() << ")\n";
		exit(1);
	}
	// wait for the solver to finish
	WaitForSingleObject(pi.hProcess, INFINITE);
}

void RunEvaluator(const std::string &fname,const std::string &policy_name)
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));
	// direct the output to the write pipe
	si.dwFlags |= STARTF_USESTDHANDLES;
	si.hStdInput = NULL;
	si.hStdOutput = s_WRITE_PIPE;
	si.hStdError = s_WRITE_PIPE;

	// run evaluator with 200 simulations
	std::string cmd = s_SOLVER_LOCATION + "\\pomdpeval.exe --simLen 50 --simNum 200 ";
	cmd += "--policy-file " + policy_name + " " + fname;
	if (!CreateProcess(NULL, const_cast<char *>(cmd.c_str()), NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi))
	{
		std::cerr << "CreateProcess failed (" << GetLastError() << ")\n";
		exit(1);
	}
	// wait for the evaluation to finish
	WaitForSingleObject(pi.hProcess, INFINITE);
}

double FindAndAdvance(const char **str)
{
	// search for the nearest number and return it
	const char *curr = *str;
	while (!isdigit(*curr))
	{
		++curr;
	}

	// if the  number is negative don't forget the minus
	if (*(curr - 1) == '-')
	{
		--curr;
	}
	return strtod(curr, const_cast<char**>(str));
}

std::vector<double> ReadReward()
{
	DWORD redword;
	std::string output;
	// read from pipe output from evaluation
	do
	{
		char buf[s_BUFFER_SIZE + 1];

		if (!ReadFile(s_READ_PIPE, buf, s_BUFFER_SIZE, &redword, 0))
		{
			std::cerr << "ReadFile failed (" << GetLastError() << ")\n";
		}

		buf[redword] = '\0';
		output += buf;
	} while (redword == s_BUFFER_SIZE);

	//default value in case of error in solver
	std::vector<double> retval{ NON_VAL, NON_VAL, NON_VAL };
	// searching for the reward (according to the format of the evaluator) if the evaluator failed return retval filled with -1000
	auto found = output.find("Finishing ...");
	if (found == std::string::npos)
		return retval;

	const char *str = &(output.c_str()[found]);

	FindAndAdvance(&str);
	FindAndAdvance(&str);
	for (int i = 0; i < 3; ++i)
		retval[i] = FindAndAdvance(&str);

	return retval;
}

int main()
{
	// init pipes
	InitPipe(&s_WRITE_PIPE, &s_READ_PIPE);

	// model initialization
	double onlineGridSize = 15;
	int gridSize = 5;
	int numRocks = 5;

	std::vector<std::pair<int, bool>> rocks;
	for (int i = 0; i < numRocks; ++i)
		rocks.emplace_back(std::make_pair(rand() % gridSize * gridSize, rand() % 2 == 0));

	RockSampleSarsop offlineModel(gridSize, rocks, onlineGridSize / gridSize);

	std::ofstream pomdp(PomdpFName, std::ios::out);
	offlineModel.SaveInFormat(pomdp);
	pomdp.close();

	RunSolver(PomdpFName, policyFName);
	RunEvaluator(PomdpFName, policyFName);
	std::vector<double> reward = ReadReward();
	// print result
	std::cout << "reward = " << reward[0] << " < " << reward[1] << ", " << reward[2] << ">\n";
	char c;
	std::cin >> c;
}