#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include "DynArray.h"
#include "IdMap.h"
#include "Entities.h"
#include "Environment.h"
#include "Heap.h"

//   Simulation.cpp   state, log, events, request reports, the per step order
//   Coordinator.cpp  assignment round and conflicts
//   Fleet.cpp        movement, work, battery and charging of the robots
//   Report.cpp       console views and the final statistics

class Simulation
{

    ofstream logFile;
    chrono::steady_clock::time_point startClock;
    bool ended;
    int *comp; // connected component of each location when every road is open
    int eventsDue;
    int doneCount;

    // Simulation.cpp
    
    void processEvents();
    void applyEvent(const Event &e);
    void reportRequests();
    bool canEverBeServed(const Request &r) const;
    void finishStep();
    void writeEnd();
    void releaseRequestOf(int robotIdx, string why);

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

public :

    string scenarioName;
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

    void prepare();

    bool openLog(string path);
    void logLine(string kind, string subject, int id, string from, string to, string detail);
    bool setRequestState(int q, int to, string detail);
    bool setRobotState(int i, int to, string detail);

    bool isFinished() const;
    void step();
    void runAll();
    void assignRound();
    void advanceRobots();

    void printEnvironment() const;
    void printRobots() const;
    void printRequests() const;
    void printRoute(int fromId, int toId);
    void printStats(ostream &out) const;
};

#endif