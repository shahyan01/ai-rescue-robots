#ifndef TYPES_H
#define TYPES_H

#include <cstring>

enum LocType { LT_JUNCTION, LT_BUILDING, LT_HOSPITAL, LT_DEPOT, LT_CHARGING, LT_COMMAND };
enum Category { CAT_MEDICAL, CAT_FIRE, CAT_SUPPLY, CAT_SEARCH, CAT_GENERAL };
enum RobotState { RS_AVAILABLE, RS_TRAVELLING, RS_PERFORMING, RS_RETURNING, RS_RECHARGING, RS_UNAVAILABLE };
enum ReqState { RQ_NOT_REPORTED, RQ_PENDING, RQ_ASSIGNED, RQ_IN_PROGRESS, RQ_COMPLETED, RQ_FAILED };
enum EventType { EV_BLOCK, EV_UNBLOCK, EV_STATION_DOWN, EV_STATION_UP, EV_SET_URGENCY, EV_ROBOT_OFFLINE, EV_ROBOT_ONLINE };

inline bool parseLocType(const char* s, LocType& out) {
	if (!strcmp(s, "JUNCTION")) { out = LT_JUNCTION; return true; }
	if (!strcmp(s, "BUILDING")) { out = LT_BUILDING; return true; }
	if (!strcmp(s, "HOSPITAL")) { out = LT_HOSPITAL; return true; }
	if (!strcmp(s, "DEPOT")) { out = LT_DEPOT; return true; }
	if (!strcmp(s, "CHARGING")) { out = LT_CHARGING; return true; }
	if (!strcmp(s, "COMMAND")) { out = LT_COMMAND; return true; }
	return false;
}

inline bool parseCategory(const char* s, bool allowGeneral, Category& out) {
	if (!strcmp(s, "MEDICAL")) { out = CAT_MEDICAL; return true; }
	if (!strcmp(s, "FIRE")) { out = CAT_FIRE; return true; }
	if (!strcmp(s, "SUPPLY")) { out = CAT_SUPPLY; return true; }
	if (!strcmp(s, "SEARCH")) { out = CAT_SEARCH; return true; }
	if (allowGeneral && !strcmp(s, "GENERAL")) { out = CAT_GENERAL; return true; }
	return false;
}

inline const char* locTypeName(LocType t) {
	switch (t) {
	case LT_JUNCTION: return "JUNCTION";
	case LT_BUILDING: return "BUILDING";
	case LT_HOSPITAL: return "HOSPITAL";
	case LT_DEPOT: return "DEPOT";
	case LT_CHARGING: return "CHARGING";
	default: return "COMMAND";
	}
}

inline const char* categoryName(Category c) {
	switch (c) {
	case CAT_MEDICAL: return "MEDICAL";
	case CAT_FIRE: return "FIRE";
	case CAT_SUPPLY: return "SUPPLY";
	case CAT_SEARCH: return "SEARCH";
	default: return "GENERAL";
	}
}

inline const char* robotStateName(RobotState s) {
	switch (s) {
	case RS_AVAILABLE: return "AVAILABLE";
	case RS_TRAVELLING: return "TRAVELLING";
	case RS_PERFORMING: return "PERFORMING";
	case RS_RETURNING: return "RETURNING";
	case RS_RECHARGING: return "RECHARGING";
	default: return "UNAVAILABLE";
	}
}

inline const char* reqStateName(ReqState s) {
	switch (s) {
	case RQ_NOT_REPORTED: return "NOT_REPORTED";
	case RQ_PENDING: return "PENDING";
	case RQ_ASSIGNED: return "ASSIGNED";
	case RQ_IN_PROGRESS: return "IN_PROGRESS";
	case RQ_COMPLETED: return "COMPLETED";
	default: return "FAILED";
	}
}

#endif
