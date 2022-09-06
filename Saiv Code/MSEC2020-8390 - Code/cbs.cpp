// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <fstream>
#include <iostream>
#include <map>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

//#include <yaml-cpp/yaml.h>

#include "cbs.hpp"
#include "timer.hpp"

using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace std;

typedef pair<int, char> pairs;

struct State {
	State(int time, int x, int y) : time(time), x(x), y(y) {}

	bool operator==(const State& s) const {
		return time == s.time && x == s.x && y == s.y;
	}

	bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

	friend std::ostream& operator<<(std::ostream& os, const State& s) {
		return os << s.time << ": (" << s.x << "," << s.y << ")";
		// return os << "(" << s.x << "," << s.y << ")";
	}

	int getX()
	{
		return x;
	}

	int getY()
	{
		return y;
	}

	int time;
	int x;
	int y;
};



namespace std {
	template <>
	struct hash<State> {
		size_t operator()(const State& s) const {
			size_t seed = 0;
			boost::hash_combine(seed, s.time);
			boost::hash_combine(seed, s.x);
			boost::hash_combine(seed, s.y);
			return seed;
		}
	};
}  // namespace std

///
enum class Action {
	Up,
	Down,
	Left,
	Right,
	Wait,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
	switch (a) {
	case Action::Up:
		os << "Up";
		break;
	case Action::Down:
		os << "Down";
		break;
	case Action::Left:
		os << "Left";
		break;
	case Action::Right:
		os << "Right";
		break;
	case Action::Wait:
		os << "Wait";
		break;
	}
	return os;
}

///

struct Conflict {
	enum Type {
		Vertex,
		Edge,
	};

	int time;
	size_t agent1;
	size_t agent2;
	Type type;

	int x1;
	int y1;
	int x2;
	int y2;

	friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
		switch (c.type) {
		case Vertex:
			return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
		case Edge:
			return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
				<< "," << c.y2 << ")";
		}
		return os;
	}
};

struct VertexConstraint {
	VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
	int time;
	int x;
	int y;

	bool operator<(const VertexConstraint& other) const {
		return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
	}

	bool operator==(const VertexConstraint& other) const {
		return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
	}

	friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
		return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
	}
};

namespace std {
	template <>
	struct hash<VertexConstraint> {
		size_t operator()(const VertexConstraint& s) const {
			size_t seed = 0;
			boost::hash_combine(seed, s.time);
			boost::hash_combine(seed, s.x);
			boost::hash_combine(seed, s.y);
			return seed;
		}
	};
}  // namespace std

struct EdgeConstraint {
	EdgeConstraint(int time, int x1, int y1, int x2, int y2)
		: time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
	int time;
	int x1;
	int y1;
	int x2;
	int y2;

	bool operator<(const EdgeConstraint& other) const {
		return std::tie(time, x1, y1, x2, y2) <
			std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
	}

	bool operator==(const EdgeConstraint& other) const {
		return std::tie(time, x1, y1, x2, y2) ==
			std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
	}

	friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
		return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
			<< "," << c.y2 << ")";
	}
};

namespace std {
	template <>
	struct hash<EdgeConstraint> {
		size_t operator()(const EdgeConstraint& s) const {
			size_t seed = 0;
			boost::hash_combine(seed, s.time);
			boost::hash_combine(seed, s.x1);
			boost::hash_combine(seed, s.y1);
			boost::hash_combine(seed, s.x2);
			boost::hash_combine(seed, s.y2);
			return seed;
		}
	};
}  // namespace std

struct Constraints {
	unordered_set <VertexConstraint> vertexConstraints;
	std::unordered_set <EdgeConstraint> edgeConstraints;

	void add(const Constraints& other) {
		vertexConstraints.insert(other.vertexConstraints.begin(),
			other.vertexConstraints.end());
		edgeConstraints.insert(other.edgeConstraints.begin(),
			other.edgeConstraints.end());
	}

	bool overlap(const Constraints& other) {
		std::vector<VertexConstraint> vertexIntersection;
		std::vector<EdgeConstraint> edgeIntersection;
		std::set_intersection(vertexConstraints.begin(), vertexConstraints.end(),
			other.vertexConstraints.begin(),
			other.vertexConstraints.end(),
			std::back_inserter(vertexIntersection));
		std::set_intersection(edgeConstraints.begin(), edgeConstraints.end(),
			other.edgeConstraints.begin(),
			other.edgeConstraints.end(),
			std::back_inserter(edgeIntersection));
		return !vertexIntersection.empty() || !edgeIntersection.empty();
	}

	friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
		for (const auto& vc : c.vertexConstraints) {
			os << vc << std::endl;
		}
		for (const auto& ec : c.edgeConstraints) {
			os << ec << std::endl;
		}
		return os;
	}
};

struct Location {
public:
	Location(int x, int y) : x(x), y(y) {}
	int x;
	int y;

	bool operator<(const Location& other) const {
		return std::tie(x, y) < std::tie(other.x, other.y);
	}

	int getX()
	{
		return x;
	}

	int getY()
	{
		return y;
	}

	bool operator==(const Location& other) const {
		return std::tie(x, y) == std::tie(other.x, other.y);
	}

	friend std::ostream& operator<<(std::ostream& os, const Location& c) {
		return os << "(" << c.x << "," << c.y << ")";
	}
};

namespace std {
	template <>
	struct hash<Location> {
		size_t operator()(const Location& s) const {
			size_t seed = 0;
			boost::hash_combine(seed, s.x);
			boost::hash_combine(seed, s.y);
			return seed;
		}
	};
}  // namespace std

///
class Environment {
public:
	Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
		std::vector<Location> goals)
		: m_dimx(dimx),
		m_dimy(dimy),
		m_obstacles(std::move(obstacles)),
		m_goals(std::move(goals)),
		m_agentIdx(0),
		m_constraints(nullptr),
		m_lastGoalConstraint(-1),
		m_highLevelExpanded(0),
		m_lowLevelExpanded(0) {
		// computeHeuristic();
	}

	Environment(const Environment&) = delete;
	Environment& operator=(const Environment&) = delete;

	void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
		assert(constraints);  // NOLINT
		m_agentIdx = agentIdx;
		m_constraints = constraints;
		m_lastGoalConstraint = -1;
		for (const auto& vc : constraints->vertexConstraints) {
			if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
				m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
			}
		}
	}

	int admissibleHeuristic(const State& s) {
		// std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
		// s.y] << std::endl;
		// return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
		return std::abs(s.x - m_goals[m_agentIdx].x) +
			std::abs(s.y - m_goals[m_agentIdx].y);
	}

	bool isSolution(const State& s) {
		return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
			s.time > m_lastGoalConstraint;
	}

	void getNeighbors(const State& s,
		std::vector<Neighbor<State, Action, int> >& neighbors) {
		// std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
		// for(const auto& vc : constraints.vertexConstraints) {
		//   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
		//   std::endl;
		// }
		neighbors.clear();
		{
			State n(s.time + 1, s.x, s.y);
			if (stateValid(n) && transitionValid(s, n)) {
				neighbors.emplace_back(
					Neighbor<State, Action, int>(n, Action::Wait, 1));
			}
		}
		{
			State n(s.time + 1, s.x - 1, s.y);
			if (stateValid(n) && transitionValid(s, n)) {
				neighbors.emplace_back(
					Neighbor<State, Action, int>(n, Action::Left, 1));
			}
		}
		{
			State n(s.time + 1, s.x + 1, s.y);
			if (stateValid(n) && transitionValid(s, n)) {
				neighbors.emplace_back(
					Neighbor<State, Action, int>(n, Action::Right, 1));
			}
		}
		{
			State n(s.time + 1, s.x, s.y + 1);
			if (stateValid(n) && transitionValid(s, n)) {
				neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
			}
		}
		{
			State n(s.time + 1, s.x, s.y - 1);
			if (stateValid(n) && transitionValid(s, n)) {
				neighbors.emplace_back(
					Neighbor<State, Action, int>(n, Action::Down, 1));
			}
		}
	}

	bool getFirstConflict(
		const std::vector<PlanResult<State, Action, int> >& solution,
		Conflict& result) {
		int max_t = 0;
		for (const auto& sol : solution) {
			max_t = std::max<int>(max_t, sol.states.size() - 1);
		}

		for (int t = 0; t < max_t; ++t) {
			// check drive-drive vertex collisions
			for (size_t i = 0; i < solution.size(); ++i) {
				State state1 = getState(i, solution, t);
				for (size_t j = i + 1; j < solution.size(); ++j) {
					State state2 = getState(j, solution, t);
					if (state1.equalExceptTime(state2)) {
						result.time = t;
						result.agent1 = i;
						result.agent2 = j;
						result.type = Conflict::Vertex;
						result.x1 = state1.x;
						result.y1 = state1.y;
						// std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
						// std::endl;
						return true;
					}
				}
			}
			// drive-drive edge (swap)
			for (size_t i = 0; i < solution.size(); ++i) {
				State state1a = getState(i, solution, t);
				State state1b = getState(i, solution, t + 1);
				for (size_t j = i + 1; j < solution.size(); ++j) {
					State state2a = getState(j, solution, t);
					State state2b = getState(j, solution, t + 1);
					if (state1a.equalExceptTime(state2b) &&
						state1b.equalExceptTime(state2a)) {
						result.time = t;
						result.agent1 = i;
						result.agent2 = j;
						result.type = Conflict::Edge;
						result.x1 = state1a.x;
						result.y1 = state1a.y;
						result.x2 = state1b.x;
						result.y2 = state1b.y;
						return true;
					}
				}
			}
		}

		return false;
	}

	void createConstraintsFromConflict(
		const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
		if (conflict.type == Conflict::Vertex) {
			Constraints c1;
			c1.vertexConstraints.emplace(
				VertexConstraint(conflict.time, conflict.x1, conflict.y1));
			constraints[conflict.agent1] = c1;
			constraints[conflict.agent2] = c1;
		}
		else if (conflict.type == Conflict::Edge) {
			Constraints c1;
			c1.edgeConstraints.emplace(EdgeConstraint(
				conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
			constraints[conflict.agent1] = c1;
			Constraints c2;
			c2.edgeConstraints.emplace(EdgeConstraint(
				conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
			constraints[conflict.agent2] = c2;
		}
	}

	void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

	void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
		int /*gScore*/) {
		m_lowLevelExpanded++;
	}

	int highLevelExpanded() { return m_highLevelExpanded; }

	int lowLevelExpanded() const { return m_lowLevelExpanded; }

private:
	State getState(size_t agentIdx,
		const std::vector<PlanResult<State, Action, int> >& solution,
		size_t t) {
		assert(agentIdx < solution.size());
		if (t < solution[agentIdx].states.size()) {
			return solution[agentIdx].states[t].first;
		}
		assert(!solution[agentIdx].states.empty());
		return solution[agentIdx].states.back().first;
	}

	bool stateValid(const State& s) {
		assert(m_constraints);
		const auto& con = m_constraints->vertexConstraints;
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
			con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
	}

	bool transitionValid(const State& s1, const State& s2) {
		assert(m_constraints);
		const auto& con = m_constraints->edgeConstraints;
		return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
			con.end();
	}
#if 0
	// We use another A* search for simplicity
	// we compute the shortest path to each goal by using the fact that our getNeighbor function is
	// symmetric and by not terminating the AStar search until the queue is empty
	void computeHeuristic()
	{
		class HeuristicEnvironment
		{
		public:
			HeuristicEnvironment(
				size_t dimx,
				size_t dimy,
				const std::unordered_set<Location>& obstacles,
				std::vector<int>* heuristic)
				: m_dimx(dimx)
				, m_dimy(dimy)
				, m_obstacles(obstacles)
				, m_heuristic(heuristic)
			{
			}

			int admissibleHeuristic(
				const Location& s)
			{
				return 0;
			}

			bool isSolution(
				const Location& s)
			{
				return false;
			}

			void getNeighbors(
				const Location& s,
				std::vector<Neighbor<Location, Action, int> >& neighbors)
			{
				neighbors.clear();

				{
					Location n(s.x - 1, s.y);
					if (stateValid(n)) {
						neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Left, 1));
					}
				}
				{
					Location n(s.x + 1, s.y);
					if (stateValid(n)) {
						neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Right, 1));
					}
				}
				{
					Location n(s.x, s.y + 1);
					if (stateValid(n)) {
						neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Up, 1));
					}
				}
				{
					Location n(s.x, s.y - 1);
					if (stateValid(n)) {
						neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Down, 1));
					}
				}
			}

			void onExpandNode(
				const Location& s,
				int fScore,
				int gScore)
			{
			}

			void onDiscover(
				const Location& s,
				int fScore,
				int gScore)
			{
				(*m_heuristic)[s.x + m_dimx * s.y] = gScore;
			}

		private:
			bool stateValid(
				const Location& s)
			{
				return    s.x >= 0
					&& s.x < m_dimx
					&& s.y >= 0
					&& s.y < m_dimy
					&& m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end();
			}

		private:
			int m_dimx;
			int m_dimy;
			const std::unordered_set<Location>& m_obstacles;
			std::vector<int>* m_heuristic;

		};

		m_heuristic.resize(m_goals.size());

		std::vector< Neighbor<State, Action, int> > neighbors;

		for (size_t i = 0; i < m_goals.size(); ++i) {
			m_heuristic[i].assign(m_dimx * m_dimy, std::numeric_limits<int>::max());
			HeuristicEnvironment henv(m_dimx, m_dimy, m_obstacles, &m_heuristic[i]);
			AStar<Location, Action, int, HeuristicEnvironment> astar(henv);
			PlanResult<Location, Action, int> dummy;
			astar.search(m_goals[i], dummy);
			m_heuristic[i][m_goals[i].x + m_dimx * m_goals[i].y] = 0;
		}
	}
#endif
private:
	int m_dimx;
	int m_dimy;
	std::unordered_set<Location> m_obstacles;
	std::vector<Location> m_goals;
	// std::vector< std::vector<int> > m_heuristic;
	size_t m_agentIdx;
	const Constraints* m_constraints;
	int m_lastGoalConstraint;
	int m_highLevelExpanded;
	int m_lowLevelExpanded;
};

struct Printer
{
public:
	Printer(int X, int Y, int ID) : x(X), y(Y), id(ID) {}
	int x;
	int y;
	int startX = x;
	int startY = y;
	int id;

	int getX()
	{
		return x;
	}
	int getY()
	{
		return y;
	}
	int getStartX()
	{
		return startX;
	}
	int getStartY()
	{
		return startY;
	}
	int getID()
	{
		return id;
	}
	void setX(int newX)
	{
		x = newX;
	}
	void setY(int newY)
	{
		y = newY;
	}

};

void print(int &cost, int phase, std::vector<PlanResult<State, Action, int>>& solution, std::vector<Location>& goalsC)
{
	std::cout << "Planning successful! " << std::endl;
	int makespan = 0;
	for (const auto& s : solution) {
		cout << "------------> " << s.cost << endl;
		cost += s.cost;
		//makespan = std::max<int>(makespan, s.cost);
	}
	int max = -1000;
	for (const auto& s : solution) {
		if (s.cost > max)
		{
			max = s.cost;
		}
	}
	

	//std::ofstream out(outputFile);
	//cout << "statistics:" << std::endl;
	//cout << "  cost: " << cost << std::endl;
	//cout << "  makespan: " << makespan << std::endl;
	//cout << "  runtime: " << timer.elapsedSeconds() << std::endl;
	//cout << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
	//cout << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
	//cout << "schedule:" << std::endl;

		// std::cout << "Solution for: " << a << std::endl;
		// for (size_t i = 0; i < solution[a].actions.size(); ++i) {
		//   std::cout << solution[a].states[i].second << ": " <<
		//   solution[a].states[i].first << "->" << solution[a].actions[i].first
		//   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
		// }
		// std::cout << solution[a].states.back().second << ": " <<
		// solution[a].states.back().first << std::endl;
	/*
	for (size_t a = 0; a < solution.size(); ++a) {
		cout << "agent" << a << ":" << std::endl;
		for (const auto& state : solution[a].states) {
			//cout << "    - x: " << state.first.x << std::endl
				//<< "      y: " << state.first.y << std::endl
				//<< "      t: " << state.second << std::endl;
			cout << "(" << state.first.x << " , " << state.first.y  << ")"<< endl;
		}
	}*/
	string nop = "phase";
	nop.append(to_string(phase));
	nop.append(".txt");
	cout << nop << endl;
	cout << "sum cost -------->" << cost << endl;
	cout << "max cost --------> " << max << endl;
	ofstream myfile;
	myfile.open(nop);
	myfile << phase << endl;
	myfile << solution.size( ) << endl;
	for (size_t a = 0; a < solution.size(); ++a) {
		myfile << a << std::endl;
		myfile << solution[a].states.size() << endl;
		for (const auto& state : solution[a].states) {
			//cout << "    - x: " << state.first.x << std::endl
				//<< "      y: " << state.first.y << std::endl
				//<< "      t: " << state.second << std::endl;
			myfile << state.first.x << " " << state.first.y << "\n" ;
		}
	}
	myfile << endl;
	myfile << "obstacles" << endl;
	myfile << goalsC.size() << endl;
	for (auto& o : goalsC) {
		myfile <<  o.getX() << " " << o.getY() << endl;
	}
	myfile.close();
}

void clearEverything(std::vector<Location>& goals, std::vector<State>& startStates, std::vector<Location>& goalsC)
{
	goals.clear();
	startStates.clear();
	goalsC.clear();
	//solution.clear();
}

/*
void runIt(int dimx, int dimy, std::unordered_set<Location>& obstacles, std::vector<State>& startStates, std::vector<Location>& goals)
{
	cout << "in Run It" << endl;
	std::vector<PlanResult<State, Action, int>> solution;
	Environment mapf(dimx, dimy, obstacles, goals);
	CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);
	Timer timer;
	bool success = cbs.search(startStates, solution);
	timer.stop();
	if (success) {
		print(solution);
	}
}
*/

void insertObstacles(std::unordered_set<Location>& obstacles, std::vector<Location>& goalsC)
{
	cout << "in insert Obstacles" << endl;
	for (Location i : goalsC)
	{
		obstacles.insert(Location(i.getX(), i.getY()));
	}
}

void iterFirst(int& cost, int phase, int (&a_mount)[20][2], int(&b_mount)[20][2], int(&c_mount)[21][2],
	int(&a_chunk)[20][2], int(&b_chunk)[20][2], int(&c_chunk)[20][2], std::vector<pairs>& present,
	std::unordered_set<Location>& obstacles, std::vector<Location>& goals, std::vector<Location>& goalsC, std::vector<State>& startStates, vector<Printer>& printerList)
{
	cout << "in iterFirst" << endl;
	//startStates.emplace_back(State(0, 1, 13));
	//startStates.emplace_back(State(0, 1, 14));
	//startStates.emplace_back(State(0, 1, 15));
	//startStates.emplace_back(State(0, 1, 16));
	for (int i = 1; i <= 2; i++)
	{
		for (auto& printy : printerList)
		{
			if (printy.getID() == i)
			{
				startStates.emplace_back(State(0, printy.getStartX(), printy.getStartY()));
				cout << "Start: " << printy.getStartX() << " " << printy.getStartY() << endl;
				
			}
		}
	}
	for (int i = 0; i < present.size(); i++)
	{
		for(auto& j : printerList)
		{
			if ((i + 1) == j.getID())
			{
				if (present.at(i).second == 'A')
				{
					goals.emplace_back(Location(a_mount[present.at(i).first - 1][1], a_mount[present.at(i).first - 1][0]));
					goalsC.emplace_back(Location(a_chunk[present.at(i).first - 1][1], a_chunk[present.at(i).first - 1][0]));
					cout << "End: " << a_mount[present.at(i).first - 1][1] << " " << a_mount[present.at(i).first - 1][0] << endl;
					printerList.at(i).setX(a_mount[present.at(i).first - 1][1]);
					printerList.at(i).setY(a_mount[present.at(i).first - 1][0]);
				}
				if (present.at(i).second == 'B')
				{
					goals.emplace_back(Location(b_mount[present.at(i).first - 1][1], b_mount[present.at(i).first - 1][0]));
					goalsC.emplace_back(Location(b_chunk[present.at(i).first - 1][1], b_chunk[present.at(i).first - 1][0]));
					cout << "End: " << b_mount[present.at(i).first - 1][1] << " " << b_mount[present.at(i).first - 1][0] << endl;
					printerList.at(i).setX(b_mount[present.at(i).first - 1][1]);
					printerList.at(i).setY(b_mount[present.at(i).first - 1][0]);
				}
				if (present.at(i).second == 'C')
				{
					goals.emplace_back(Location(c_mount[present.at(i).first - 1][1], c_mount[present.at(i).first - 1][0]));
					goalsC.emplace_back(Location(c_chunk[present.at(i).first - 1][1], c_chunk[present.at(i).first - 1][0]));
					cout << "End: " << c_mount[present.at(i).first - 1][1] << " " << c_mount[present.at(i).first - 1][0] << endl;
					printerList.at(i).setX(c_mount[present.at(i).first - 1][1]);
					printerList.at(i).setY(c_mount[present.at(i).first - 1][0]);
				}
			}
		}
	}
	//for (auto& g : goals)
	//{
	//	cout << g.getX() << " " << g.getY() << endl;
	//}
	//runIt(20, 20, obstacles, startStates, goals);
	cout << "in Run It" << endl;
	std::vector<PlanResult<State, Action, int>> solution;
	Environment mapf(20, 20, obstacles, goals);
	CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);
	Timer timer;
	bool success = cbs.search(startStates, solution);
	timer.stop();
	if (success) {
		print(cost, phase, solution, goalsC);
	}
	else
	{
		cout << "Error in run it" << endl;
	}
	insertObstacles(obstacles, goalsC);
	clearEverything(goals, startStates, goalsC);
	//for (const auto& s : solution) {
		//cost += s.cost;
		//makespan = std::max<int>(makespan, s.cost);
	//}
}

void iterRest(int& cost, int phase, int(&a_mount)[20][2], int(&b_mount)[20][2], int(&c_mount)[21][2],
	int(&a_chunk)[20][2], int(&b_chunk)[20][2], int(&c_chunk)[20][2], std::vector<pairs>& present,
	std::unordered_set<Location>& obstacles, std::vector<Location>& goals, std::vector<Location>& goalsC, std::vector<State>& startStates, vector<Printer>& printerList)
{
	cout << endl;
	//for (auto itr = obstacles.begin(); itr != obstacles.end(); ++itr) {
		//cout << "obstacles: " << (*itr) << " " << endl;
	//}
	/*for (auto& p : printerList)
	{
		cout << "Printer: "  << p.getID() << " " << p.getX() << " " << p.getY() << endl;
	}*/
	//startStates.emplace_back(State(0, printerList.at(0).getStartX(), printerList.at(0).getY()));
	//startStates.emplace_back(State(0, printerList.at(1).getStartX(), printerList.at(1).getY()));
	//startStates.emplace_back(State(0, printerList.at(2).getStartX(), printerList.at(2).getY()));
	//startStates.emplace_back(State(0, printerList.at(3).getStartX(), printerList.at(3).getY()));
	int length = present.size();
	int antiL = 4 - length;
	int len = 4 - antiL;
	cout << len << " " << antiL << endl;
	for (int iy = 1; iy <= len; iy++) 
	{
		for (auto& cody : printerList)
		{
			if (cody.getID() == iy)
			{
				startStates.emplace_back(State(0, cody.getX(), cody.getY()));
				//cout << cody.getX() << " " << cody.getY() << endl;
			}
		}
	}
	
	for (int it = 4; it > len; it--)
	{
		for (auto& cod : printerList)
		{
			if (cod.getID() == it)
			{
				//if (cod.getStartX() != cod.getX() && cod.getStartY() != cod.getY())
				//{
				startStates.emplace_back(State(0, cod.getX(), cod.getY()));
				//goals.emplace_back(Location(cod.getStartX(), cod.getStartY()));
				cod.setX(cod.getStartX());
				cod.setY(cod.getStartY());
				//cout << cod.getX() << " " << cod.getY() << endl;
				//}
			}
		}
	}

	for (int i = 0; i < present.size(); i++)
	{
		for (auto& j : printerList)
		{
			if ((i + 1) == j.getID())
			{
				
					if (present.at(i).second == 'A')
					{
						goals.emplace_back(Location(a_mount[present.at(i).first - 1][1], a_mount[present.at(i).first - 1][0]));
						goalsC.emplace_back(Location(a_chunk[present.at(i).first - 1][1], a_chunk[present.at(i).first - 1][0]));
						printerList.at(i).setX(a_mount[present.at(i).first - 1][1]);
						printerList.at(i).setY(a_mount[present.at(i).first - 1][0]);
					}
					if (present.at(i).second == 'B')
					{
						goals.emplace_back(Location(b_mount[present.at(i).first - 1][1], b_mount[present.at(i).first - 1][0]));
						goalsC.emplace_back(Location(b_chunk[present.at(i).first - 1][1], b_chunk[present.at(i).first - 1][0]));
						printerList.at(i).setX(b_mount[present.at(i).first - 1][1]);
						printerList.at(i).setY(b_mount[present.at(i).first - 1][0]);
					}
					if (present.at(i).second == 'C')
					{
						goals.emplace_back(Location(c_mount[present.at(i).first - 1][1], c_mount[present.at(i).first - 1][0]));
						goalsC.emplace_back(Location(c_chunk[present.at(i).first - 1][1], c_chunk[present.at(i).first - 1][0]));
						printerList.at(i).setX(c_mount[present.at(i).first - 1][1]);
						printerList.at(i).setY(c_mount[present.at(i).first - 1][0]);
					}
				
			}
		}
	}

	for (int it = 4; it > length; it--)
	{
		for (auto& cod : printerList)
		{
			if (cod.getID() == it)
			{
				//if (cod.getStartX() != cod.getX() && cod.getStartY() != cod.getY())
				//{
					//startStates.emplace_back(State(0, cod.getX(), cod.getY()));
					goals.emplace_back(Location(cod.getStartX(), cod.getStartY()));
					//cod.setX(cod.getStartX());
					//cod.setY(cod.getStartY());
				//}
			}
		}
	}
	
	//cout << "size: " << goals.size() << endl;
	//for (auto& s : startStates)
	//{
		//cout << "start: " << s.getX() << " " << s.getY() << endl;
	//}
	//for (auto& g : goals)
	//{
		//cout << "goals: " << g.getX() << " " << g.getY() << endl;
	//}
	/*for (auto& p : printerList)
	{
		cout << "Printer: " << p.getID() << " " << p.getX() << " " << p.getY() << endl;
	}*/
	//runIt(20, 20, obstacles, startStates, goals);
	//cout << "in Run It" << endl;
	std::vector<PlanResult<State, Action, int>> solution;
	Environment mapf(20, 20, obstacles, goals);
	CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);
	Timer timer;
	bool success = cbs.search(startStates, solution);
	timer.stop();
	if (success) {
		print(cost, phase, solution, goalsC);
	}
	else
	{
		cout << "Error in run it" << endl;
	}
	insertObstacles(obstacles, goalsC);
	clearEverything(goals, startStates, goalsC);
	//for (const auto& s : solution) {
		//cost += s.cost;
		//makespan = std::max<int>(makespan, s.cost);
	//}
}

int main(int argc, char* argv[])
{
	
	//goals.emplace_back(Location(a_mount[][], a_mount[][]));
	//goals.emplace_back(Location(b_mount[][], b_mount[][]));
	//goals.emplace_back(Location(c_mount[][], c_mount[][]));
	int dimx = 20;
	int dimy = 20;
	int cost = 0;
	int a_mount[20][2] = {
		{3, 13},
		{3, 15},
		{3, 14},
		{3, 16},
		{2, 13},
		{2, 15},
		{2, 14},
		{2, 16},
		{1, 13},
		{1, 15},
		{1, 14},
		{1, 16},
		{6, 13},
		{6, 15},
		{6, 14},
		{6, 16},
		{7, 13},
		{7, 15},
		{7, 14},
		{7, 16}
	}; //  //chunk[]
	int a_chunk[20][2] = {
			{4, 13},
			{4, 15},
			{4, 14},
			{4, 16},
			{3, 13},
			{3, 15},
			{3, 14},
			{3, 16},
			{2, 13},
			{2, 15},
			{2, 14},
			{2, 16},
			{5, 13},
			{5, 15},
			{5, 14},
			{5, 16},
			{6, 13},
			{6, 15},
			{6, 14},
			{6, 16}
	};
	int b_mount[20][2] = {
		{13, 13},
		{13, 15},
		{13, 14},
		{13, 16},
		{12, 13},
		{12, 15},
		{12, 14},
		{12, 16},
		{11, 13},
		{11, 15},
		{11, 14},
		{11, 16},
		{16, 13},
		{16, 15},
		{16, 14},
		{16, 16},
		{17, 13},
		{17, 15},
		{17, 14},
		{17, 16}
	};
	int b_chunk[20][2] = {
			{14, 13},
			{14, 15},
			{14, 14},
			{14, 16},
			{13, 13},
			{13, 15},
			{13, 14},
			{13, 16},
			{12, 13},
			{12, 15},
			{12, 14},
			{12, 16},
			{15, 13},
			{15, 15},
			{15, 14},
			{15, 16},
			{16, 13},
			{16, 15},
			{16, 14},
			{16, 16}
	};
	int c_mount[21][2] = {
		{5, 3},
		{5, 5},
		{3, 4},
		{3, 6}, 

	{2, 3},
	{2, 5},
	{6, 3},
	{6, 5},

	{2, 4},
	{2, 6},
	{6, 4},
	{6, 6},

	{1, 3},
	{1, 5},
	{7, 3},
	{7, 5},

	{1, 4},
	{1, 6},
	{7, 4},
	{7, 6},
	{13, 1}
	};
	int c_chunk[20][2] = {
		{4, 3},
		{4, 5},
		{4, 4},
		{4, 6},

	{3, 3},
	{3, 5},
	{5, 3},
	{5, 5},

	{3, 4},
	{3, 6},
	{5, 4},
	{5, 6},

	{2, 3},
	{2, 5},
	{6, 3},
	{6, 5},

	{2, 4},
	{2, 6},
	{6, 4},
	{6, 6}
	};
	// chunk [number - 1][0 for X, 1 for Y]
	std::unordered_set<Location> obstacles;
	std::vector<Location> goals;
	std::vector<Location> goalsC;
	std::vector<State> startStates;
	vector<Printer> printerList;
	printerList.emplace_back(1, 13, 1);
	printerList.emplace_back(1, 14, 2);
	printerList.emplace_back(1, 15, 3);
	printerList.emplace_back(1, 16, 4);
	int starts[4][2] = { {1, 13}, {1, 14}, {1, 15}, {1, 16}};
	//phase planning
	vector<vector<pairs>> sced;
	int q = 5;
	if (q == 1)
	{
		//phase 1
		vector<pairs> temp;
		temp.emplace_back(make_pair(1, 'A'));
		temp.emplace_back(make_pair(2, 'A'));
		temp.emplace_back(make_pair(1, 'B'));
		temp.emplace_back(make_pair(2, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 2
		temp.emplace_back(make_pair(1, 'C'));
		temp.emplace_back(make_pair(2, 'C'));
		temp.emplace_back(make_pair(3, 'A'));
		temp.emplace_back(make_pair(3, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 3
		temp.emplace_back(make_pair(4, 'B'));
		temp.emplace_back(make_pair(3, 'C'));
		temp.emplace_back(make_pair(4, 'C'));
		temp.emplace_back(make_pair(4, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 4
		temp.emplace_back(make_pair(5, 'A'));
		temp.emplace_back(make_pair(5, 'B'));
		temp.emplace_back(make_pair(5, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 5
		temp.emplace_back(make_pair(6, 'A'));
		temp.emplace_back(make_pair(6, 'B'));
		temp.emplace_back(make_pair(6, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 6
		temp.emplace_back(make_pair(7, 'A'));
		temp.emplace_back(make_pair(7, 'B'));
		temp.emplace_back(make_pair(7, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 7
		temp.emplace_back(make_pair(8, 'A'));
		temp.emplace_back(make_pair(8, 'B'));
		temp.emplace_back(make_pair(8, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 8
		temp.emplace_back(make_pair(9, 'A'));
		temp.emplace_back(make_pair(9, 'B'));
		temp.emplace_back(make_pair(9, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 9
		temp.emplace_back(make_pair(10, 'A'));
		temp.emplace_back(make_pair(10, 'B'));
		temp.emplace_back(make_pair(10, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 10
		temp.emplace_back(make_pair(11, 'A'));
		temp.emplace_back(make_pair(12, 'A'));
		temp.emplace_back(make_pair(11, 'B'));
		temp.emplace_back(make_pair(13, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 11
		temp.emplace_back(make_pair(11, 'C'));
		temp.emplace_back(make_pair(13, 'C'));
		temp.emplace_back(make_pair(13, 'A'));
		temp.emplace_back(make_pair(14, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 12
		temp.emplace_back(make_pair(12, 'B'));
		temp.emplace_back(make_pair(14, 'B'));
		temp.emplace_back(make_pair(12, 'C'));
		temp.emplace_back(make_pair(14, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 13
		temp.emplace_back(make_pair(15, 'A'));
		temp.emplace_back(make_pair(15, 'B'));
		temp.emplace_back(make_pair(16, 'B'));
		temp.emplace_back(make_pair(15, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 14
		temp.emplace_back(make_pair(16, 'C'));
		temp.emplace_back(make_pair(16, 'A'));
		temp.emplace_back(make_pair(17, 'A'));
		temp.emplace_back(make_pair(17, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 15
		temp.emplace_back(make_pair(18, 'B'));
		temp.emplace_back(make_pair(17, 'C'));
		temp.emplace_back(make_pair(18, 'C'));
		temp.emplace_back(make_pair(18, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 16
		temp.emplace_back(make_pair(19, 'B'));
		temp.emplace_back(make_pair(19, 'C'));
		temp.emplace_back(make_pair(19, 'A'));
		temp.emplace_back(make_pair(20, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 17
		temp.emplace_back(make_pair(20, 'B'));
		temp.emplace_back(make_pair(20, 'C'));
		sced.emplace_back(temp);
		temp.clear();
	} 
	if (q == 2)
	{
		//phase 1
		vector<pairs> temp;
		temp.emplace_back(make_pair(1, 'A'));
		temp.emplace_back(make_pair(2, 'A'));
		temp.emplace_back(make_pair(1, 'B'));
		temp.emplace_back(make_pair(2, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 2
		temp.emplace_back(make_pair(3, 'A'));
		temp.emplace_back(make_pair(1, 'C'));
		temp.emplace_back(make_pair(3, 'B'));
		temp.emplace_back(make_pair(4, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 3
		temp.emplace_back(make_pair(4, 'A'));
		temp.emplace_back(make_pair(5, 'A'));
		temp.emplace_back(make_pair(5, 'B'));
		temp.emplace_back(make_pair(2, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 4
		temp.emplace_back(make_pair(6, 'A'));
		temp.emplace_back(make_pair(3, 'C'));
		temp.emplace_back(make_pair(6, 'B'));
		temp.emplace_back(make_pair(4, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 5
		temp.emplace_back(make_pair(7, 'A'));
		temp.emplace_back(make_pair(5, 'C'));
		temp.emplace_back(make_pair(7, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 6
		temp.emplace_back(make_pair(8, 'A'));
		temp.emplace_back(make_pair(6, 'C'));
		temp.emplace_back(make_pair(8, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 7
		temp.emplace_back(make_pair(9, 'A'));
		temp.emplace_back(make_pair(7, 'C'));
		temp.emplace_back(make_pair(9, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 8
		temp.emplace_back(make_pair(10, 'A'));
		temp.emplace_back(make_pair(8, 'C'));
		temp.emplace_back(make_pair(10, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 9
		temp.emplace_back(make_pair(11, 'A'));
		temp.emplace_back(make_pair(12, 'A'));
		temp.emplace_back(make_pair(11, 'B'));
		temp.emplace_back(make_pair(13, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 10
		temp.emplace_back(make_pair(13, 'A'));
		temp.emplace_back(make_pair(14, 'A'));
		temp.emplace_back(make_pair(12, 'B'));
		temp.emplace_back(make_pair(14, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 11
		temp.emplace_back(make_pair(15, 'A'));
		temp.emplace_back(make_pair(9, 'C'));
		temp.emplace_back(make_pair(15, 'B'));
		temp.emplace_back(make_pair(16, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 12
		temp.emplace_back(make_pair(16, 'A'));
		temp.emplace_back(make_pair(17, 'A'));
		temp.emplace_back(make_pair(17, 'B'));
		temp.emplace_back(make_pair(18, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 13
		temp.emplace_back(make_pair(18, 'A'));
		temp.emplace_back(make_pair(10, 'C'));
		temp.emplace_back(make_pair(19, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		// phase 14
		temp.emplace_back(make_pair(19, 'A'));
		temp.emplace_back(make_pair(20, 'A'));
		temp.emplace_back(make_pair(20, 'B'));
		temp.emplace_back(make_pair(11, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		// phase 15
		temp.emplace_back(make_pair(13, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		// phase 16
		temp.emplace_back(make_pair(12, 'C'));
		temp.emplace_back(make_pair(14, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		// phase 17
		temp.emplace_back(make_pair(15, 'C'));
		temp.emplace_back(make_pair(16, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		// phase 18
		temp.emplace_back(make_pair(17, 'C'));
		temp.emplace_back(make_pair(18, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		// phase 19
		temp.emplace_back(make_pair(19, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 20
		temp.emplace_back(make_pair(20, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase 21
		temp.emplace_back(make_pair(21, 'C'));
		sced.emplace_back(temp);
		temp.clear();
	}
	if (q == 3)
	{
		//phase1
		vector<pairs> temp;
		temp.emplace_back(make_pair(1, 'A'));
		temp.emplace_back(make_pair(2, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase2
		
		temp.emplace_back(make_pair(3, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase3
		
		temp.emplace_back(make_pair(4, 'A'));
		temp.emplace_back(make_pair(5, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase4
		
		temp.emplace_back(make_pair(6, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase5
		
		temp.emplace_back(make_pair(7, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase6
		
		temp.emplace_back(make_pair(8, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase7
		
		temp.emplace_back(make_pair(9, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase8
		
		temp.emplace_back(make_pair(10, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase9
		
		temp.emplace_back(make_pair(11, 'A'));
		temp.emplace_back(make_pair(12, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase10
		
		temp.emplace_back(make_pair(13, 'A'));
		temp.emplace_back(make_pair(14, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase11
		
		temp.emplace_back(make_pair(15, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase12
		
		temp.emplace_back(make_pair(16, 'A'));
		temp.emplace_back(make_pair(17, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase13
		
		temp.emplace_back(make_pair(18, 'A'));
		sced.emplace_back(temp);
		temp.clear();
		//phase14
		
		temp.emplace_back(make_pair(19, 'A'));
		temp.emplace_back(make_pair(20, 'A'));
		sced.emplace_back(temp);
		temp.clear();
	}
	if (q == 4)
	{
		//phase1
		vector<pairs> temp;
		temp.emplace_back(make_pair(1, 'B'));
		temp.emplace_back(make_pair(2, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase2
		temp.emplace_back(make_pair(3, 'B'));
		temp.emplace_back(make_pair(4, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase3
		temp.emplace_back(make_pair(5, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase4

		temp.emplace_back(make_pair(6, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase5

		temp.emplace_back(make_pair(7, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase6

		temp.emplace_back(make_pair(8, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase7

		temp.emplace_back(make_pair(9, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase8

		temp.emplace_back(make_pair(10, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase9

		temp.emplace_back(make_pair(11, 'B'));
		temp.emplace_back(make_pair(13, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase10

		temp.emplace_back(make_pair(12, 'B'));
		temp.emplace_back(make_pair(14, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase11

		temp.emplace_back(make_pair(15, 'B'));
		temp.emplace_back(make_pair(16, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase12

		
		temp.emplace_back(make_pair(17, 'B'));
		temp.emplace_back(make_pair(18, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase13

		temp.emplace_back(make_pair(19, 'B'));
		sced.emplace_back(temp);
		temp.clear();
		//phase14
		temp.emplace_back(make_pair(20, 'B'));
		sced.emplace_back(temp);
		temp.clear();
	}
	if (q == 5)
	{
		//phase1
		vector<pairs> temp;
		temp.emplace_back(make_pair(1, 'C'));
		temp.emplace_back(make_pair(2, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase2
		temp.emplace_back(make_pair(3, 'C'));
		temp.emplace_back(make_pair(4, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase3
		temp.emplace_back(make_pair(5, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase4

		temp.emplace_back(make_pair(6, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase5

		temp.emplace_back(make_pair(7, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase6

		temp.emplace_back(make_pair(8, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase7

		temp.emplace_back(make_pair(9, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase8

		temp.emplace_back(make_pair(10, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase9

		temp.emplace_back(make_pair(11, 'C'));
		temp.emplace_back(make_pair(13, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase10

		temp.emplace_back(make_pair(12, 'C'));
		temp.emplace_back(make_pair(14, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase11

		temp.emplace_back(make_pair(15, 'C'));
		temp.emplace_back(make_pair(16, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase12


		temp.emplace_back(make_pair(17, 'C'));
		temp.emplace_back(make_pair(18, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase13

		temp.emplace_back(make_pair(19, 'C'));
		sced.emplace_back(temp);
		temp.clear();
		//phase14
		temp.emplace_back(make_pair(20, 'C'));
		sced.emplace_back(temp);
		temp.clear();
	}
	//iterFirst(a_mount, b_mount, c_mount, a_chunk, b_chunk, c_chunk, sced.at(0), obstacles, goals, goalsC, startStates, printerList);
	//phase execution
	
	for (int i = 0; i < sced.size(); i++) 
	{
		if (i == 0)
		{
			iterFirst(cost, (i+1), a_mount, b_mount, c_mount, a_chunk, b_chunk, c_chunk, sced.at(0), obstacles, goals, goalsC, startStates, printerList);
		}
		else
		{
			iterRest( cost, (i+1), a_mount, b_mount, c_mount, a_chunk, b_chunk, c_chunk, sced.at(i), obstacles, goals, goalsC, startStates, printerList);
		}
	}
	cout << "---------------------------";
	cout << "total cost: " << cost;
	cout << "---------------------------";
	
	/*
	//startStates
	startStates.emplace_back(State(0, 1, 13));
	startStates.emplace_back(State(0, 1, 14));
	startStates.emplace_back(State(0, 1, 15));
	startStates.emplace_back(State(0, 1, 16));
	//Goals
	goals.emplace_back(Location(a_mount[0][1], a_mount[0][0]));
	goals.emplace_back(Location(a_mount[1][1], a_mount[1][0]));
	goals.emplace_back(Location(b_mount[0][1], b_mount[0][0]));
	goals.emplace_back(Location(b_mount[1][1], b_mount[1][0]));
	//Obstacles to Add
	goalsC.emplace_back(Location(a_chunk[0][1], a_chunk[0][0]));
	goalsC.emplace_back(Location(a_chunk[1][1], a_chunk[1][0]));
	goalsC.emplace_back(Location(b_chunk[0][1], b_chunk[0][0]));
	goalsC.emplace_back(Location(b_chunk[1][1], b_chunk[1][0]));
	runIt(dimx, dimy, obstacles, startStates, goals);
	insertObstacles(obstacles, goalsC);
	clearEverything(goals, startStates);
	//startStates
	startStates.emplace_back(State(0,a_mount[0][1], a_mount[0][0]));
	startStates.emplace_back(State(0,a_mount[1][1], a_mount[1][0]));
	startStates.emplace_back(State(0,b_mount[0][1], b_mount[0][0]));
	startStates.emplace_back(State(0,b_mount[1][1], b_mount[1][0]));
	//
	goals.emplace_back(Location(c_mount[0][1], c_mount[0][0]));
	goals.emplace_back(Location(c_mount[1][1], c_mount[1][0]));
	goals.emplace_back(Location(a_mount[2][1], a_mount[2][0]));
	goals.emplace_back(Location(b_mount[2][1], b_mount[2][0]));
	runIt(dimx, dimy, obstacles, startStates, goals);
	*/
	return 0;
}