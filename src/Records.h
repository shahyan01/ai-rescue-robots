#ifndef RECORDS_H
#define RECORDS_H

// No functions, only data.

struct AdjNode
{
	int to; // location index
	int conn; // connection index
	AdjNode *next;
};

struct Location
{
	int id;
	char name[32];
	int type;
	bool stationUp;
	AdjNode *firstRoad;
};

struct Connection
{
	int id;
	int aId;
	int bId;
	int aIdx;
	int bIdx;
	int cost;
	bool blocked;
};

struct Robot
{
	int id;
	int location; // start location
	int capability;
	int maxBattery;
	int battery;
	int state;
	bool online;

	int at; // location the robot is standing on
	int request; // request it holds, or -1
	int roadTo;
	int roadRemain; // cost units left
	int workLeft; // work steps left
	int expectRemain; // route cost expected to be left
	bool warned; // low battery warning
	int travelSteps;
	int workSteps;
	int onlineSteps;
	int consumed;
	int recharged;
	int moved;
};

struct Request
{
	int id;
	int reportTime;
	int location;       // location id
	int category;
	int urgency;
	int duration;
	int state;

	int assignedRobot;  // robot index
	int place;          // location index
	int heapPos;        // position in the pending heap, -1 when not in it
	int arriveTime;     // step a robot last arrived here, -1 if never
	int doneTime;
};

struct Event
{
	int time;
	int type;
	int a;
	int b;
};

struct Params
{
	int maxSteps;
	int speed;
	int batteryPerCost;
	int batteryPerWork;
	int rechargePerStep;
	int lowBatteryPercent;
};

struct Stats
{
	int eventsApplied;
	int eventsRejected;
	int requestsReported;
	int assignments;
	int conflicts;
	int reassignments;
	int badTransitions;
};

#endif