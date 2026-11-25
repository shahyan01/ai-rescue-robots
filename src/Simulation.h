#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <fstream>
#include <ctime>
#include "Constants.h"
#include "Records.h"
#include "Environment.h"
#include "Heap.h"

using namespace std;

// The simulation is split over four files:
//   Simulation.cpp   setup, log, events, request reports, order of a step
//   Coordinator.cpp  the assignment round
//   Fleet.cpp        movement, work, battery and charging of the robots
//   Report.cpp       the menu views and the final statistics
class Simulation {
private:
	ofstream logFile;
	clock_t startClock;
	bool ended;
	int* component;     // which connected part of the map each location is in (all roads open)
	int eventsDue;      // events that can still happen (time < max_steps)
	int doneCount;      // requests that are COMPLETED or FAILED

	// Simulation.cpp
	void processEvents();
	void applyEvent(Event& e);
	void reportRequests();
	bool canEverBeServed(Request& r);
	void finishStep();
	void writeEnd();
	void releaseRequestOf(int robotIdx, const char* why);

	// Coordinator.cpp
	bool isIdle(int robotIdx);

	// Fleet.cpp
	void spend(Robot& b, int amount);
	void startRoad(Robot& b, int conn, int to);
	bool moveAlongRoad(Robot& b);
	bool atWorkingStation(Robot& b);
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
	Robot* robots;
	int numRobots;
	Request* requests;
	int numRequests;
	Event* events;
	int numEvents;
	PendingHeap pending;
	Stats stats;
	int now;
	int endStep;
	int nextEvent;
	int nextRequest;

	Simulation();
	~Simulation();

	void reserveRobots(int n);
	void reserveRequests(int n);
	void reserveEvents(int n);
	int findRobot(int id);
	int findRequest(int id);

	// Called by the loader when everything has been read and checked.
	void prepare();

	bool openLog(const char* path);
	void logStart(const char* kind, const char* subject, int id, const char* from, const char* to);
	bool setRequestState(int q, int to, const char* detail);
	bool setRobotState(int i, int to, const char* detail);

	bool isFinished();
	void step();
	void runAll();
	void assignRound();
	void advanceRobots();

	void printEnvironment();
	void printRobots();
	void printRequests();
	void printRoute(int fromId, int toId);
	void printStats(ostream& out);
};

#endif
