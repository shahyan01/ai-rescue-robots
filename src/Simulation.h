#ifndef SIMULATION_H
#define SIMULATION_H

#include <cstdio>
#include <ctime>
#include "DynArray.h"
#include "IdMap.h"
#include "Entities.h"
#include "Environment.h"
#include "Heap.h"

// The simulation is split over four files:
//   Simulation.cpp   state, log, events, request reports, the per-step order
//   Coordinator.cpp  assignment round and conflicts
//   Fleet.cpp        movement, work, battery and charging of the robots
//   Report.cpp       console views and the final statistics
class Simulation {
private:
	FILE* logFile;
	clock_t startClock;
	bool ended;
	int* comp;          // connected component of each location when every road is open
	int eventsDue;      // events that can still fire (time < max_steps)
	int doneCount;      // requests that are COMPLETED or FAILED

	Simulation(const Simulation&);
	Simulation& operator=(const Simulation&);

	// Simulation.cpp
	void processEvents();
	void applyEvent(const Event& e);
	void reportRequests();
	bool canEverBeServed(const Request& r) const;
	void finishStep();
	void writeEnd();
	void releaseRequestOf(int robotIdx, const char* why);

	// Coordinator.cpp
	bool isIdle(int robotIdx) const;

	// Fleet.cpp
	void spend(Robot& b, int amount);
	void startRoad(Robot& b, int conn, int to);
	bool moveAlongRoad(Robot& b);
	bool atWorkingStation(const Robot& b) const;
	void travel(int i);
	void arrive(int i);
	void doWork(int i);
	void returnTrip(int i);
	void idleStep(int i);
	void rechargeStep(int i);

public:
	char scenarioName[64];
	Params params;
	Environment env;
	DynArray<Robot> robots;
	DynArray<Request> requests;
	DynArray<Event> events;
	IdMap robotMap;
	IdMap requestMap;
	PendingHeap pending;
	Stats stats;
	int now;
	int endStep;
	int nextEvent;
	int nextRequest;

	Simulation();
	~Simulation();

	// Called by the loader once everything has been parsed and validated.
	void prepare();

	bool openLog(const char* path);
	void logLine(const char* kind, const char* subject, int id, const char* from, const char* to, const char* fmt, ...);
	bool setRequestState(int q, ReqState to, const char* detail);
	bool setRobotState(int i, RobotState to, const char* detail);

	bool isFinished() const { return ended; }
	void step();
	void runAll();
	void assignRound();
	void advanceRobots();

	void printEnvironment() const;
	void printRobots() const;
	void printRequests() const;
	void printRoute(int fromId, int toId);
	void printStats(FILE* out) const;
};

#endif
