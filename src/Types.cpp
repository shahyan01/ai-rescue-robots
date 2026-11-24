#include "Types.h"

bool parseLocType(string s, int& out) {
    if (s == "JUNCTION") { out = LT_JUNCTION; return true; }
    if (s == "BUILDING") { out = LT_BUILDING; return true; }
    if (s == "HOSPITAL") { out = LT_HOSPITAL; return true; }
    if (s == "DEPOT") { out = LT_DEPOT; return true; }
    if (s == "CHARGING") { out = LT_CHARGING; return true; }
    if (s == "COMMAND") { out = LT_COMMAND; return true; }
    return false;
}

bool parseCategory(string s, bool allowGeneral, int& out) {
    if (s == "MEDICAL") { out = CAT_MEDICAL; return true; }
    if (s == "FIRE") { out = CAT_FIRE; return true; }
    if (s == "SUPPLY") { out = CAT_SUPPLY; return true; }
    if (s == "SEARCH") { out = CAT_SEARCH; return true; }
    if (allowGeneral && s == "GENERAL") { out = CAT_GENERAL; return true; }
    return false;
}

string locTypeName(int t) {
    if (t == LT_JUNCTION) return "JUNCTION";
    if (t == LT_BUILDING) return "BUILDING";
    if (t == LT_HOSPITAL) return "HOSPITAL";
    if (t == LT_DEPOT) return "DEPOT";
    if (t == LT_CHARGING) return "CHARGING";
    return "COMMAND";
}

string categoryName(int c) {
    if (c == CAT_MEDICAL) return "MEDICAL";
    if (c == CAT_FIRE) return "FIRE";
    if (c == CAT_SUPPLY) return "SUPPLY";
    if (c == CAT_SEARCH) return "SEARCH";
    return "GENERAL";
}

string robotStateName(int s) {
    if (s == RS_AVAILABLE) return "AVAILABLE";
    if (s == RS_TRAVELLING) return "TRAVELLING";
    if (s == RS_PERFORMING) return "PERFORMING";
    if (s == RS_RETURNING) return "RETURNING";
    if (s == RS_RECHARGING) return "RECHARGING";
    return "UNAVAILABLE";
}

string reqStateName(int s) {
    if (s == RQ_NOT_REPORTED) return "NOT_REPORTED";
    if (s == RQ_PENDING) return "PENDING";
    if (s == RQ_ASSIGNED) return "ASSIGNED";
    if (s == RQ_IN_PROGRESS) return "IN_PROGRESS";
    if (s == RQ_COMPLETED) return "COMPLETED";
    return "FAILED";
}
