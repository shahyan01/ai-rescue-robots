#ifndef TYPES_H
#define TYPES_H

#include <string>

using namespace std;

// Location types

const int LT_JUNCTION = 0;
const int LT_BUILDING = 1;
const int LT_HOSPITAL = 2;
const int LT_DEPOT = 3;
const int LT_CHARGING = 4;
const int LT_COMMAND = 5;

// Categories

const int CAT_MEDICAL = 0;
const int CAT_FIRE = 1;
const int CAT_SUPPLY = 2;
const int CAT_SEARCH = 3;
const int CAT_GENERAL = 4;

// Robot states

const int RS_AVAILABLE = 0;
const int RS_TRAVELLING = 1;
const int RS_PERFORMING = 2;
const int RS_RETURNING = 3;
const int RS_RECHARGING = 4;
const int RS_UNAVAILABLE = 5;

// Request states

const int RQ_NOT_REPORTED = 0;
const int RQ_PENDING = 1;
const int RQ_ASSIGNED = 2;
const int RQ_IN_PROGRESS = 3;
const int RQ_COMPLETED = 4;
const int RQ_FAILED = 5;

// Event types

const int EV_BLOCK = 0;
const int EV_UNBLOCK = 1;
const int EV_STATION_DOWN = 2;
const int EV_STATION_UP = 3;
const int EV_SET_URGENCY = 4;
const int EV_ROBOT_OFFLINE = 5;
const int EV_ROBOT_ONLINE = 6;

bool parseLocType(string s, int &out);
bool parseCategory(string s, bool allowGeneral, int &out);

string locTypeName(int t);
string categoryName(int c);
string robotStateName(int s);
string reqStateName(int s);

#endif