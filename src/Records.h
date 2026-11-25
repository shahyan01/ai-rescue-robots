#ifndef RECORDS_H
#define RECORDS_H

// Plain records. No functions in here, only data.

struct AdjNode {
	int to;          // location index at the other end of the road
	int conn;        // connection index
	AdjNode* next;
};

struct Location {
	int id;
	char name[32];
	int type;
	bool stationUp;
	AdjNode* firstRoad;
};

struct Connection {
	int id;
	int aId;
	int bId;
	int aIdx;
	int bIdx;
	int cost;
	bool blocked;
};

struct Robot {
	int id;
	int location;       // start location id as written in the file
	int capability;
	int maxBattery;
	int battery;
	int state;
	bool online;

	int at;             // index of the location the robot is standing on
	int request;        // index of the request it holds, or -1
	int roadTo;         // location index at the end of the current road, -1 when standing on a location
	int roadRemain;     // cost units left on the current road
	int workLeft;       // work steps left on the current request
	int expectRemain;   // route cost expected to be left, used to spot a re-route (-1 = none)
	bool warned;        // low-battery warning already written
	int travelSteps;
	int workSteps;
	int onlineSteps;
	int consumed;
	int recharged;
	int moved;
};

struct Request {
	int id;
	int reportTime;
	int location;       // location id as written in the file
	int category;
	int urgency;
	int duration;
	int state;

	int assignedRobot;  // robot index or -1
	int place;          // location index
	int heapPos;        // position in the pending heap, -1 when not in it
	int arriveTime;     // step a robot last arrived here, -1 if never
	int doneTime;
};

struct Event {
	int time;
	int type;
	int a;
	int b;
};

struct Params {
	int maxSteps;
	int speed;
	int batteryPerCost;
	int batteryPerWork;
	int rechargePerStep;
	int lowBatteryPercent;
};

struct Stats {
	int eventsApplied;
	int eventsRejected;
	int requestsReported;
	int assignments;
	int conflicts;
	int reassignments;
	int badTransitions;
};

#endif
