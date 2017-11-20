#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "./core/globals.h"
#include "./core/pomdp.h"
#include "./pomdpx/pomdpx.h"
#include "./ippc/client.h"
#include "./util/util.h"

namespace despot {

static std::vector<double> &operator+=(std::vector<double> &vec, const std::vector<double> &toAdd)
{
	if (vec.size() < toAdd.size())
		vec.resize(toAdd.size(), 0);

	for (int i = 0; i < toAdd.size(); ++i)
			vec[i] += toAdd[i];

	return vec;
}

static std::vector<double> &operator/=(std::vector<double> &vec, int toDivide)
{
	for (int i = 0; i < vec.size(); ++i)
		vec[i] /= toDivide;

	return vec;
}

static std::vector<std::vector<double>> &operator+=(std::vector<std::vector<double>> &vec, const std::vector<std::vector<double>> &toAdd)
{
	if (vec.size() < toAdd.size())
		vec.resize(toAdd.size());

	for (int i = 0; i < toAdd.size(); ++i)
		vec[i] += toAdd[i];

	return vec;
}

static std::vector<std::vector<double>> &operator/=(std::vector<std::vector<double>> &vec, int toDivide)
{
	for (int i = 0; i < vec.size(); ++i)
		vec[i] /= toDivide;

	return vec;
}

class Tree_Properties // NATAN CHANGES
{
public:
	Tree_Properties() : m_height(0), m_size(0), m_levelSize(), m_levelActionSize(), m_preferredActionPortion() {};

	void UpdateCount();
	static void ZeroCount();
	void Avg();

	double m_height;
	double m_size;
	std::vector<double> m_levelSize;
	std::vector<std::vector<double>> m_levelActionSize;
	std::vector<double> m_preferredActionPortion;

	static std::vector<int> s_levelCounter;
};

inline void Tree_Properties::UpdateCount()
{
	if (s_levelCounter.size() < m_levelSize.size())
		s_levelCounter.resize(m_levelSize.size(), 0);
	
	for (int i = 0; i < m_levelSize.size(); ++i)
		++s_levelCounter[i];
}

inline void Tree_Properties::ZeroCount()
{
	s_levelCounter.resize(0);
}

inline void Tree_Properties::Avg()
{
	if (s_levelCounter.size() == 0)
		return;

	int numTrees = s_levelCounter[0];
	m_height /= numTrees;
	m_size /= numTrees;
	m_levelSize /= numTrees;
	m_levelActionSize /= numTrees;

	for (int i = 0; i < m_preferredActionPortion.size(); ++i)
		m_preferredActionPortion[i] /= s_levelCounter[i];
}

inline Tree_Properties &operator+=(Tree_Properties &p1, Tree_Properties &p2) // NATAN CHANGES
{
	p1.m_height += p2.m_height;
	p1.m_size += p2.m_size;
	p1.m_levelSize += p2.m_levelSize;
	p1.m_levelActionSize += p2.m_levelActionSize;
	p1.m_preferredActionPortion += p2.m_preferredActionPortion;

	return p1;
}

inline Tree_Properties &operator/=(Tree_Properties &p1, int toDivide) // NATAN CHANGES
{
	p1.m_height /= toDivide;
	p1.m_size /= toDivide;
	p1.m_levelSize /= toDivide;
	p1.m_levelActionSize /= toDivide;
	p1.m_preferredActionPortion /= toDivide;

	return p1;
}
/* =============================================================================
 * EvalLog class
 * =============================================================================*/

class EvalLog {
private:
  std::vector<std::string> runned_instances;
	std::vector<int> num_of_completed_runs;
	std::string log_file_;

public:
	static time_t start_time;

	static double curr_inst_start_time;
	static double curr_inst_target_time; // Targetted amount of time used for each step
	static double curr_inst_budget; // Total time in seconds given for current instance
	static double curr_inst_remaining_budget; // Remaining time in seconds for current instance
	static int curr_inst_steps;
	static int curr_inst_remaining_steps;
	static double allocated_time;
	static double plan_time_ratio;

	EvalLog(std::string log_file);

	void Save();
	void IncNumOfCompletedRuns(std::string problem);
	int GetNumCompletedRuns() const;
	int GetNumRemainingRuns() const;
	int GetNumCompletedRuns(std::string instance) const;
	int GetNumRemainingRuns(std::string instance) const;
	double GetUsedTimeInSeconds() const;
	double GetRemainingTimeInSeconds() const;

	// Pre-condition: curr_inst_start_time is initialized
	void SetInitialBudget(std::string instance);
	double GetRemainingBudget(std::string instance) const;
};

/* =============================================================================
 * Evaluator class
 * =============================================================================*/

/** Interface for evaluating a solver's performance by simulating how it runs
 * in a real.
 */
class Evaluator {
protected:
	DSPOMDP* model_;
	std::string belief_type_;
	Solver* solver_;
	clock_t start_clockt_;
	State* state_;
	int step_;
	double target_finish_time_;
	std::ostream* out_;

	std::vector<double> discounted_round_rewards_;
	std::vector<double> undiscounted_round_rewards_;
	double reward_;
	double total_discounted_reward_;
	double total_undiscounted_reward_;
	std::vector<Tree_Properties> tree_properties_;// NATAN CHANGES
	std::shared_ptr<std::ofstream> stream_to_save_tree_;

public:
	Evaluator(DSPOMDP* model, std::string belief_type, Solver* solver,
		clock_t start_clockt, std::ostream* out);
	virtual ~Evaluator();
	
	void InitTreeFile(std::shared_ptr<std::ofstream> treeFile) { stream_to_save_tree_ = treeFile; }
	inline void out(std::ostream* o) {
		out_ = o;
	}

	inline void rewards(std::vector<double> rewards) {
		undiscounted_round_rewards_ = rewards;
	}

	inline std::vector<double> rewards() {
		return undiscounted_round_rewards_;
	}

	inline int step() {
		return step_;
	}
	inline double target_finish_time() {
		return target_finish_time_;
	}
	inline void target_finish_time(double t) {
		target_finish_time_ = t;
	}
	inline Solver* solver() {
		return solver_;
	}
	inline void solver(Solver* s) {
		solver_ = s;
	}
	inline DSPOMDP* model() {
		return model_;
	}
	inline void model(DSPOMDP* m) {
		model_ = m;
	}

	virtual inline void world_seed(unsigned seed) {
	}

	virtual int Handshake(std::string instance) = 0; // Initialize simulator and return number of runs.
	virtual void InitRound() = 0;

	bool RunStep(int step, int round);

	virtual double EndRound() = 0; // Return total undiscounted reward for this round.
	virtual bool ExecuteAction(int action, double& reward, OBS_TYPE& obs) = 0;
	virtual void ReportStepReward();
	virtual double End() = 0; // Free resources and return total reward collected

	virtual void UpdateTimePerMove(double step_time) = 0;

	double AverageUndiscountedRoundReward() const;
	double StderrUndiscountedRoundReward() const;
	double AverageDiscountedRoundReward() const;
	double StderrDiscountedRoundReward() const;

	void AddDiscounted2String(std::string & buffer) const;//NATAN CHANGES
	void AddUnDiscounted2String(std::string & buffer) const;

	void ResizeTreeProp(int size) { tree_properties_.resize(size); }
	void AvgTreeProp(int idx) {tree_properties_[idx].Avg();};

	void PrintTreeProp(std::string &buffer) const;
};

/* =============================================================================
 * IPPCEvaluator class
 * =============================================================================*/

/** Evaluation protocol used in IPPC'11 and IPPC'14. */
class IPPCEvaluator: public Evaluator {
private:
	POMDPX* pomdpx_;
	Client* client_;
	EvalLog log_;
	std::string instance_;
	std::string hostname_;
	std::string port_;

public:
	IPPCEvaluator(DSPOMDP* model, std::string belief_type, Solver* solver,
		clock_t start_clockt, std::string hostname, std::string port, std::string log,
		std::ostream* out);
	~IPPCEvaluator();

	void port_number(std::string port);
	int GetNumCompletedRuns() const {
		return log_.GetNumCompletedRuns();
	}
	int GetNumCompletedRuns(std::string instance) const {
		return log_.GetNumCompletedRuns(instance);
	}

	int Handshake(std::string instance);
	void InitRound();
	double EndRound();
	bool ExecuteAction(int action, double& reward, OBS_TYPE& obs);
	// void ReportStepReward();
	double End();
	void UpdateTimeInfo(std::string instance);
	void UpdateTimePerMove(double step_time);
};

/* =============================================================================
 * POMDPEvaluator class
 * =============================================================================*/

/** Evaluation by simulating using a DSPOMDP model.*/
class POMDPEvaluator: public Evaluator {
protected:
	Random random_;

public:
	POMDPEvaluator(DSPOMDP* model, std::string belief_type, Solver* solver,
		clock_t start_clockt, std::ostream* out, double target_finish_time = -1,
		int num_steps = -1);
	~POMDPEvaluator();

	virtual inline void world_seed(unsigned seed) {
		random_ = Random(seed);
	}

	int Handshake(std::string instance);
	void InitRound();
	double EndRound();
	bool ExecuteAction(int action, double& reward, OBS_TYPE& obs);
	double End();
	void UpdateTimePerMove(double step_time);
};

} // namespace despot

#endif
