#include "ScenarioLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>

void ErrorList::add(string text) {
    msgs.push(text);
}

int ErrorList::total() const {
    return msgs.size();
}

void ErrorList::print(string title) const {
    cout << title << " (" << msgs.size() << "):" << endl;
    for (int i = 0; i < msgs.size(); i++)
        cout << "  - " << msgs[i] << endl;
}

const int SEC_NONE = 0;
const int SEC_LOC = 1;
const int SEC_CONN = 2;
const int SEC_ROB = 3;
const int SEC_REQ = 4;
const int SEC_EVT = 5;
const int SEC_SKIP = 6;

const int MAX_TOK = 16;

// Splits a line into words.
int tokenize(string line, string tok[]) {
    istringstream in(line);
    int n = 0;
    while (n < MAX_TOK && in >> tok[n])
        n++;
    return n;
}

bool toInt(string s, int& v) {
    if (s.length() == 0)
        return false;

    int start = 0;
    if (s[0] == '-' || s[0] == '+')
        start = 1;
    if (start == (int)s.length())
        return false;

    long long x = 0;
    for (int i = start; i < (int)s.length(); i++) {
        if (s[i] < '0' || s[i] > '9')
            return false;
        x = x * 10 + (s[i] - '0');
        if (x > 1000000000)
            return false;
    }
    if (s[0] == '-')
        x = -x;
    v = (int)x;
    return true;
}

string sectionName(int s) {
    if (s == SEC_LOC) return "LOCATIONS";
    if (s == SEC_CONN) return "CONNECTIONS";
    if (s == SEC_ROB) return "ROBOTS";
    if (s == SEC_REQ) return "REQUESTS";
    if (s == SEC_EVT) return "EVENTS";
    return "";
}

bool isKeyword(string s) {
    return s == "SCENARIO" || s == "PARAM" || s == "LOCATIONS" || s == "CONNECTIONS" ||
        s == "ROBOTS" || s == "REQUESTS" || s == "EVENTS" || s == "END";
}

void parseLocation(string tok[], int n, int ln, Simulation& sim, ErrorList& errs) {
    Location l;
    string at = "line " + to_string(ln) + ": ";
    if (n != 3) {
        errs.add(at + "location needs 3 fields (id name type), found " + to_string(n));
        return;
    }
    if (!toInt(tok[0], l.id)) {
        errs.add(at + "location id '" + tok[0] + "' is not a number");
        return;
    }
    l.name = tok[1];
    if (!parseLocType(tok[2], l.type)) {
        errs.add(at + "unknown location type '" + tok[2] + "'");
        return;
    }
    l.stationUp = true;
    l.adjHead = -1;
    sim.env.locs.push(l);
}

void parseConnection(string tok[], int n, int ln, Simulation& sim, ErrorList& errs) {
    Connection c;
    string at = "line " + to_string(ln) + ": ";
    if (n != 5) {
        errs.add(at + "connection needs 5 fields (id a b cost status), found " + to_string(n));
        return;
    }
    if (!toInt(tok[0], c.id) || !toInt(tok[1], c.aId) || !toInt(tok[2], c.bId) || !toInt(tok[3], c.cost)) {
        errs.add(at + "connection id, endpoints and cost must be whole numbers");
        return;
    }
    if (c.cost < 1) {
        errs.add(at + "connection " + to_string(c.id) + " has cost " + to_string(c.cost) + ", cost must be at least 1");
        return;
    }
    if (tok[4] == "OPEN")
        c.blocked = false;
    else if (tok[4] == "BLOCKED")
        c.blocked = true;
    else {
        errs.add(at + "connection status '" + tok[4] + "' must be OPEN or BLOCKED");
        return;
    }
    c.aIdx = -1;
    c.bIdx = -1;
    sim.env.conns.push(c);
}

void parseRobot(string tok[], int n, int ln, Simulation& sim, ErrorList& errs) {
    Robot r;
    string at = "line " + to_string(ln) + ": ";
    if (n != 5) {
        errs.add(at + "robot needs 5 fields (id start capability max initial), found " + to_string(n));
        return;
    }
    if (!toInt(tok[0], r.id) || !toInt(tok[1], r.location) || !toInt(tok[3], r.maxBattery) || !toInt(tok[4], r.battery)) {
        errs.add(at + "robot id, start and battery values must be whole numbers");
        return;
    }
    if (!parseCategory(tok[2], true, r.capability)) {
        errs.add(at + "unknown robot capability '" + tok[2] + "'");
        return;
    }
    if (r.maxBattery < 1 || r.battery < 0 || r.battery > r.maxBattery) {
        errs.add(at + "robot " + to_string(r.id) + " has battery " + to_string(r.battery) + " with max " +
            to_string(r.maxBattery) + " (need 0 <= initial <= max, max >= 1)");
        return;
    }
    r.state = RS_AVAILABLE;
    r.online = true;
    sim.robots.push(r);
}

void parseRequest(string tok[], int n, int ln, Simulation& sim, ErrorList& errs) {
    Request r;
    string at = "line " + to_string(ln) + ": ";
    if (n != 6) {
        errs.add(at + "request needs 6 fields (id time location category urgency duration), found " + to_string(n));
        return;
    }
    if (!toInt(tok[0], r.id) || !toInt(tok[1], r.reportTime) || !toInt(tok[2], r.location) ||
        !toInt(tok[4], r.urgency) || !toInt(tok[5], r.duration)) {
        errs.add(at + "request numeric fields must be whole numbers");
        return;
    }
    if (!parseCategory(tok[3], false, r.category)) {
        errs.add(at + "unknown request category '" + tok[3] + "'");
        return;
    }
    if (r.reportTime < 0 || r.urgency < 1 || r.urgency > 5 || r.duration < 1) {
        errs.add(at + "request " + to_string(r.id) + " needs time >= 0, urgency 1-5 and duration >= 1");
        return;
    }
    r.state = RQ_NOT_REPORTED;
    r.assignedRobot = -1;
    sim.requests.push(r);
}

void parseEvent(string tok[], int n, int ln, Simulation& sim, ErrorList& errs) {
    Event e;
    e.b = 0;
    string at = "line " + to_string(ln) + ": ";
    if (n < 3) {
        errs.add(at + "event needs at least 3 fields (time TYPE target)");
        return;
    }
    if (!toInt(tok[0], e.time) || e.time < 0) {
        errs.add(at + "event time '" + tok[0] + "' must be a whole number >= 0");
        return;
    }

    int expected = 3;
    if (tok[1] == "BLOCK") e.type = EV_BLOCK;
    else if (tok[1] == "UNBLOCK") e.type = EV_UNBLOCK;
    else if (tok[1] == "STATION_DOWN") e.type = EV_STATION_DOWN;
    else if (tok[1] == "STATION_UP") e.type = EV_STATION_UP;
    else if (tok[1] == "ROBOT_OFFLINE") e.type = EV_ROBOT_OFFLINE;
    else if (tok[1] == "ROBOT_ONLINE") e.type = EV_ROBOT_ONLINE;
    else if (tok[1] == "SET_URGENCY") {
        e.type = EV_SET_URGENCY;
        expected = 4;
    }
    else {
        errs.add(at + "unknown event type '" + tok[1] + "'");
        return;
    }

    if (n != expected) {
        errs.add(at + tok[1] + " needs " + to_string(expected) + " fields, found " + to_string(n));
        return;
    }
    if (!toInt(tok[2], e.a) || (expected == 4 && !toInt(tok[3], e.b))) {
        errs.add(at + "event target must be a whole number");
        return;
    }
    if (e.type == EV_SET_URGENCY && (e.b < 1 || e.b > 5)) {
        errs.add(at + "new urgency " + to_string(e.b) + " must be 1-5");
        return;
    }
    sim.events.push(e);
}

void parseParam(string tok[], int n, int ln, Simulation& sim, ErrorList& errs, bool& haveMaxSteps) {
    int v;
    string at = "line " + to_string(ln) + ": ";
    if (n != 3 || !toInt(tok[2], v)) {
        errs.add(at + "PARAM needs a key and a whole-number value");
        return;
    }

    if (tok[1] == "max_steps") {
        if (v < 1) {
            errs.add(at + "max_steps must be at least 1");
            return;
        }
        sim.params.maxSteps = v;
        haveMaxSteps = true;
    }
    else if (tok[1] == "speed") {
        if (v < 1) {
            errs.add(at + "speed must be at least 1");
            return;
        }
        sim.params.speed = v;
    }
    else if (tok[1] == "battery_per_cost_unit") {
        if (v < 0) {
            errs.add(at + "battery_per_cost_unit cannot be negative");
            return;
        }
        sim.params.batteryPerCost = v;
    }
    else if (tok[1] == "battery_per_work_step") {
        if (v < 0) {
            errs.add(at + "battery_per_work_step cannot be negative");
            return;
        }
        sim.params.batteryPerWork = v;
    }
    else if (tok[1] == "recharge_per_step") {
        if (v < 0) {
            errs.add(at + "recharge_per_step cannot be negative");
            return;
        }
        sim.params.rechargePerStep = v;
    }
    else if (tok[1] == "low_battery_percent") {
        if (v < 0 || v > 100) {
            errs.add(at + "low_battery_percent must be 0 to 100");
            return;
        }
        sim.params.lowBatteryPercent = v;
    }
    // unknown keys are ignored so that extended files still load
}

void closeSection(int sec, int remaining, int declared, int ln, ErrorList& errs) {
    if (sec != SEC_NONE && sec != SEC_SKIP && remaining > 0)
        errs.add("line " + to_string(ln) + ": " + sectionName(sec) + " declares " + to_string(declared) +
            " records but only " + to_string(declared - remaining) + " were found");
}

// Stable merge sort by time. Records with equal time keep their file order.
void sortEvents(DynArray<Event>& a) {
    int n = a.size();
    Event* tmp = new Event[n + 1];
    for (int width = 1; width < n; width *= 2) {
        for (int lo = 0; lo < n; lo += 2 * width) {
            int mid = n;
            if (lo + width < n)
                mid = lo + width;
            int hi = n;
            if (lo + 2 * width < n)
                hi = lo + 2 * width;

            int i = lo;
            int j = mid;
            int k = lo;
            while (i < mid && j < hi) {
                if (a[j].time < a[i].time)
                    tmp[k++] = a[j++];
                else
                    tmp[k++] = a[i++];
            }
            while (i < mid)
                tmp[k++] = a[i++];
            while (j < hi)
                tmp[k++] = a[j++];
        }
        for (int i = 0; i < n; i++)
            a[i] = tmp[i];
    }
    delete[] tmp;
}

// Stable merge sort by report time.
void sortRequests(DynArray<Request>& a) {
    int n = a.size();
    Request* tmp = new Request[n + 1];
    for (int width = 1; width < n; width *= 2) {
        for (int lo = 0; lo < n; lo += 2 * width) {
            int mid = n;
            if (lo + width < n)
                mid = lo + width;
            int hi = n;
            if (lo + 2 * width < n)
                hi = lo + 2 * width;

            int i = lo;
            int j = mid;
            int k = lo;
            while (i < mid && j < hi) {
                if (a[j].reportTime < a[i].reportTime)
                    tmp[k++] = a[j++];
                else
                    tmp[k++] = a[i++];
            }
            while (i < mid)
                tmp[k++] = a[i++];
            while (j < hi)
                tmp[k++] = a[j++];
        }
        for (int i = 0; i < n; i++)
            a[i] = tmp[i];
    }
    delete[] tmp;
}

void validate(Simulation& sim, ErrorList& errs, ErrorList& warns) {
    Environment& env = sim.env;
    int bad = 0;

    bool locOk = env.indexLocations(bad);
    if (!locOk)
        errs.add("location id " + to_string(bad) + " is duplicated or out of range (0 to " + to_string(IdMap::MAX_ID) + ")");
    bool connOk = env.indexConnections(bad);
    if (!connOk)
        errs.add("connection id " + to_string(bad) + " is duplicated or out of range (0 to " + to_string(IdMap::MAX_ID) + ")");

    int n = sim.robots.size();
    int* ids = new int[n + 1];
    for (int i = 0; i < n; i++)
        ids[i] = sim.robots[i].id;
    if (!sim.robotMap.build(ids, n, bad))
        errs.add("robot id " + to_string(bad) + " is duplicated or out of range");
    delete[] ids;

    sortRequests(sim.requests);
    n = sim.requests.size();
    ids = new int[n + 1];
    for (int i = 0; i < n; i++)
        ids[i] = sim.requests[i].id;
    if (!sim.requestMap.build(ids, n, bad))
        errs.add("request id " + to_string(bad) + " is duplicated or out of range");
    delete[] ids;

    if (locOk) {
        for (int i = 0; i < env.conns.size(); i++) {
            const Connection& c = env.conns[i];
            if (env.locIndex(c.aId) == -1 || env.locIndex(c.bId) == -1)
                errs.add("connection " + to_string(c.id) + " refers to a location that does not exist (" +
                    to_string(c.aId) + ", " + to_string(c.bId) + ")");
            else if (c.aId == c.bId)
                errs.add("connection " + to_string(c.id) + " connects location " + to_string(c.aId) + " to itself");
        }
        for (int i = 0; i < sim.robots.size(); i++)
            if (env.locIndex(sim.robots[i].location) == -1)
                errs.add("robot " + to_string(sim.robots[i].id) + " starts at location " +
                    to_string(sim.robots[i].location) + " which does not exist");
        for (int i = 0; i < sim.requests.size(); i++)
            if (env.locIndex(sim.requests[i].location) == -1)
                errs.add("request " + to_string(sim.requests[i].id) + " is at location " +
                    to_string(sim.requests[i].location) + " which does not exist");
    }

    sortEvents(sim.events);
    for (int i = 0; i < sim.events.size(); i++) {
        const Event& e = sim.events[i];
        string at = "event at time " + to_string(e.time) + " ";

        if (e.type == EV_BLOCK || e.type == EV_UNBLOCK) {
            if (connOk && env.connIndex(e.a) == -1)
                errs.add(at + "refers to unknown connection " + to_string(e.a));
        }
        else if (e.type == EV_STATION_DOWN || e.type == EV_STATION_UP) {
            if (locOk) {
                int li = env.locIndex(e.a);
                if (li == -1)
                    errs.add(at + "refers to unknown location " + to_string(e.a));
                else if (env.locs[li].type != LT_CHARGING)
                    errs.add("event at time " + to_string(e.time) + ": location " + to_string(e.a) + " is not a charging station");
            }
        }
        else if (e.type == EV_SET_URGENCY) {
            if (sim.requestMap.find(e.a) == -1)
                errs.add(at + "refers to unknown request " + to_string(e.a));
        }
        else if (e.type == EV_ROBOT_OFFLINE || e.type == EV_ROBOT_ONLINE) {
            if (sim.robotMap.find(e.a) == -1)
                errs.add(at + "refers to unknown robot " + to_string(e.a));
        }
    }

    // Warnings: things that are legal but will never have an effect.
    int nr = sim.requests.size();
    int* urg = new int[nr + 1];
    for (int i = 0; i < nr; i++)
        urg[i] = sim.requests[i].urgency;
    for (int i = 0; i < sim.events.size(); i++) {
        const Event& e = sim.events[i];
        if (e.time >= sim.params.maxSteps)
            warns.add("event " + to_string(i + 1) + " at time " + to_string(e.time) + " is after max_steps " +
                to_string(sim.params.maxSteps) + " and will never fire");
        if (e.type == EV_SET_URGENCY) {
            int q = sim.requestMap.find(e.a);
            if (q != -1) {
                if (urg[q] == e.b)
                    warns.add("event at time " + to_string(e.time) + " sets request " + to_string(e.a) +
                        " to urgency " + to_string(e.b) + ", which it already has");
                urg[q] = e.b;
            }
        }
    }
    for (int i = 0; i < nr; i++)
        if (sim.requests[i].reportTime >= sim.params.maxSteps)
            warns.add("request " + to_string(sim.requests[i].id) + " is reported at time " +
                to_string(sim.requests[i].reportTime) + ", after max_steps " + to_string(sim.params.maxSteps) +
                ", and will never be reported");
    delete[] urg;
}

bool loadScenario(string path, Simulation& sim, ErrorList& errs, ErrorList& warns) {
    ifstream file(path.c_str());
    if (!file) {
        errs.add("cannot open file '" + path + "'");
        return false;
    }

    string line;
    string tok[MAX_TOK];
    int ln = 0;
    int sec = SEC_NONE;
    int remaining = 0;
    int declared = 0;
    bool seen[7] = {false, false, false, false, false, false, false};
    bool haveMaxSteps = false;
    bool sawEnd = false;

    while (getline(file, line)) {
        ln++;

        // skip blank lines and comments
        int first = line.find_first_not_of(" \t\r\n");
        if (first == (int)string::npos || line[first] == '#')
            continue;

        if (sawEnd) {
            errs.add("line " + to_string(ln) + ": text after END");
            break;
        }

        int n = tokenize(line, tok);
        if (n == 0)
            continue;

        if (isKeyword(tok[0])) {
            closeSection(sec, remaining, declared, ln, errs);
            sec = SEC_NONE;
            remaining = 0;

            if (tok[0] == "END") {
                sawEnd = true;
            }
            else if (tok[0] == "SCENARIO") {
                if (n >= 2)
                    sim.scenarioName = tok[1];
            }
            else if (tok[0] == "PARAM") {
                parseParam(tok, n, ln, sim, errs, haveMaxSteps);
            }
            else {
                int s = SEC_EVT;
                if (tok[0] == "LOCATIONS") s = SEC_LOC;
                else if (tok[0] == "CONNECTIONS") s = SEC_CONN;
                else if (tok[0] == "ROBOTS") s = SEC_ROB;
                else if (tok[0] == "REQUESTS") s = SEC_REQ;

                int cnt;
                if (n != 2 || !toInt(tok[1], cnt) || cnt < 0) {
                    errs.add("line " + to_string(ln) + ": " + tok[0] + " header needs a count (e.g. " + tok[0] + " 10)");
                } else {
                    if (seen[s])
                        errs.add("line " + to_string(ln) + ": section " + tok[0] + " appears twice");
                    seen[s] = true;
                    sec = s;
                    declared = cnt;
                    remaining = cnt;
                }
            }
            continue;
        }

        // A line that looks like a section header but is not a known one.
        int cnt2;
        bool upper = true;
        for (int i = 0; i < (int)tok[0].length(); i++)
            if (tok[0][i] < 'A' || tok[0][i] > 'Z')
                upper = false;
        if ((sec == SEC_NONE || remaining == 0) && n == 2 && upper && toInt(tok[1], cnt2) && cnt2 >= 0) {
            errs.add("line " + to_string(ln) + ": unknown section '" + tok[0] + "'");
            sec = SEC_SKIP;
            remaining = cnt2;
            continue;
        }

        if (sec == SEC_SKIP && remaining > 0) {
            remaining--;
            continue;
        }

        if (sec == SEC_NONE || remaining == 0) {
            errs.add("line " + to_string(ln) + ": unexpected line '" + tok[0] +
                "' (more records than the section count, or unknown keyword)");
            continue;
        }

        remaining--;
        if (sec == SEC_LOC) parseLocation(tok, n, ln, sim, errs);
        else if (sec == SEC_CONN) parseConnection(tok, n, ln, sim, errs);
        else if (sec == SEC_ROB) parseRobot(tok, n, ln, sim, errs);
        else if (sec == SEC_REQ) parseRequest(tok, n, ln, sim, errs);
        else if (sec == SEC_EVT) parseEvent(tok, n, ln, sim, errs);
    }
    file.close();

    if (!sawEnd) {
        closeSection(sec, remaining, declared, ln, errs);
        errs.add("file ended without an END line");
    }
    if (!haveMaxSteps)
        errs.add("PARAM max_steps is missing");
    if (!seen[SEC_LOC]) errs.add("section LOCATIONS is missing");
    if (!seen[SEC_CONN]) errs.add("section CONNECTIONS is missing");
    if (!seen[SEC_ROB]) errs.add("section ROBOTS is missing");

    // Cross-checks always run, so every problem in the file is reported in one go.
    validate(sim, errs, warns);

    if (errs.total() > 0)
        return false;

    sim.env.buildAdjacency();
    sim.prepare();
    return true;
}
