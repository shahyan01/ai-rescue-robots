#ifndef ENTITIES_H
#define ENTITIES_H

#include "Types.h"

struct Robot
{
    int id;
    int location; // start location
    int capability;
    int maxBattery;
    int battery;
    int state;
    bool online;

    int at;
    int request; // request it holds
    int roadTo; // location index, -1 when standing on a location
    int roadRemain; // cost units left on the road
    int workLeft; // work left on the request
    int expectRemain; // route cost expected to be left and     used to spot a re route (-1 = none)
    bool warned; // low battery warning already logged
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
    int location; // location id
    int category;
    int urgency;
    int duration;
    int state;

    int assignedRobot; // robot index
    int place; // location index
    int heapPos; // position in the pending heap
    int arriveTime; // robot last arrived
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