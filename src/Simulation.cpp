#include "Simulation.h"

Simulation::Simulation() {
    scenarioName = "";
    ended = false;
    comp = 0;
    eventsDue = 0;
    doneCount = 0;

    params.maxSteps = 0;
    params.speed = 1;
    params.batteryPerCost = 1;
    params.batteryPerWork = 2;
    params.rechargePerStep = 5;
    params.lowBatteryPercent = 30;

    stats.eventsApplied = 0;
    stats.eventsRejected = 0;
    stats.requestsReported = 0;
    stats.assignments = 0;
    stats.conflicts = 0;
    stats.reassignments = 0;
    stats.badTransitions = 0;

    now = 0;
    endStep = 0;
    nextEvent = 0;
    nextRequest = 0;
    startClock = chrono::steady_clock::now();
}

Simulation::~Simulation() {
    delete[] comp;
}

bool Simulation::isFinished() const {
    return ended;
}

void Simulation::prepare() {
    int n = env.locs.size();

    // Connected components with every road treated as open. Used to decide
    // whether a request can ever be reached by a robot.
    comp = new int[n + 1];
    int* stack = new int[n + 1];
    for (int i = 0; i < n; i++)
        comp[i] = -1;
    int label = 0;
    for (int s = 0; s < n; s++) {
        if (comp[s] != -1)
            continue;
        comp[s] = label;
        int top = 0;
        stack[top] = s;
        top++;
        while (top > 0) {
            top--;
            int u = stack[top];
            for (int e = env.locs[u].adjHead; e != -1; e = env.adj[e].next) {
                int v = env.adj[e].to;
                if (comp[v] == -1) {
                    comp[v] = label;
                    stack[top] = v;
                    top++;
                }
            }
        }
        label++;
    }
    delete[] stack;

    for (int i = 0; i < robots.size(); i++) {
        Robot& b = robots[i];
        b.at = env.locIndex(b.location);
        b.state = RS_AVAILABLE;
        b.online = true;
        b.request = -1;
        b.roadTo = -1;
        b.roadRemain = 0;
        b.workLeft = 0;
        b.expectRemain = -1;
        b.warned = false;
        b.travelSteps = 0;
        b.workSteps = 0;
        b.onlineSteps = 0;
        b.consumed = 0;
        b.recharged = 0;
        b.moved = 0;
    }
    for (int i = 0; i < requests.size(); i++) {
        Request& r = requests[i];
        r.place = env.locIndex(r.location);
        r.state = RQ_NOT_REPORTED;
        r.assignedRobot = -1;
        r.heapPos = -1;
        r.arriveTime = -1;
        r.doneTime = -1;
    }
    pending.init(&requests);

    eventsDue = 0;
    for (int i = 0; i < events.size(); i++)
        if (events[i].time < params.maxSteps)
            eventsDue++;

    startClock = chrono::steady_clock::now();
}

bool Simulation::openLog(string path) {
    logFile.open(path.c_str());
    if (!logFile.is_open())
        return false;
    logFile << "step,kind,subject,id,from,to,detail\n";
    logLine("EVENT", "simulation", 0, "-", "RUNNING",
        "scenario=" + scenarioName +
        " max_steps=" + to_string(params.maxSteps) +
        " locations=" + to_string(env.locs.size()) +
        " connections=" + to_string(env.conns.size()) +
        " robots=" + to_string(robots.size()) +
        " requests=" + to_string(requests.size()) +
        " events=" + to_string(events.size()));
    return true;
}

void Simulation::logLine(string kind, string subject, int id, string from, string to, string detail) {
    if (!logFile.is_open())
        return;
    logFile << now << "," << kind << "," << subject << "," << id << ","
            << from << "," << to << "," << detail << "\n";
}

// Only the transitions of the request state machine are allowed.
// Anything else is rejected and logged.
bool Simulation::setRequestState(int q, int to, string detail) {
    Request& r = requests[q];
    int from = r.state;
    bool ok = false;

    if (from == RQ_NOT_REPORTED)
        ok = (to == RQ_PENDING);
    else if (from == RQ_PENDING)
        ok = (to == RQ_ASSIGNED || to == RQ_FAILED);
    else if (from == RQ_ASSIGNED)
        ok = (to == RQ_IN_PROGRESS || to == RQ_PENDING);
    else if (from == RQ_IN_PROGRESS)
        ok = (to == RQ_COMPLETED || to == RQ_PENDING);

    if (!ok) {
        logLine("REJECT", "request", r.id, reqStateName(from), reqStateName(to), "invalid transition");
        stats.badTransitions++;
        return false;
    }
    r.state = to;
    logLine("STATE", "request", r.id, reqStateName(from), reqStateName(to), detail);
    return true;
}

bool Simulation::setRobotState(int i, int to, string detail) {
    Robot& b = robots[i];
    int from = b.state;
    if (from == to)
        return true;

    bool ok = false;
    if (to == RS_UNAVAILABLE)
        ok = true;
    else if (from == RS_AVAILABLE)
        ok = (to == RS_TRAVELLING || to == RS_RETURNING || to == RS_RECHARGING);
    else if (from == RS_TRAVELLING)
        ok = (to == RS_PERFORMING || to == RS_AVAILABLE);
    else if (from == RS_PERFORMING)
        ok = (to == RS_AVAILABLE);
    else if (from == RS_RETURNING)
        ok = (to == RS_RECHARGING || to == RS_AVAILABLE);
    else if (from == RS_RECHARGING)
        ok = (to == RS_AVAILABLE || to == RS_TRAVELLING);
    else if (from == RS_UNAVAILABLE)
        ok = (to == RS_AVAILABLE);

    if (!ok) {
        logLine("REJECT", "robot", b.id, robotStateName(from), robotStateName(to), "invalid transition");
        stats.badTransitions++;
        return false;
    }
    b.state = to;
    b.warned = false;
    logLine("STATE", "robot", b.id, robotStateName(from), robotStateName(to), detail);
    return true;
}

// One time step. The order never changes:
//   1. events due at this step, in file order
//   2. requests reported at this step, in file order
//   3. assignment round (robots that went offline already gave their request back in step 1)
//   4. robots move, work or recharge, and re-plan when their next road is blocked
//   5. count the step and check whether the run is over
void Simulation::step() {
    if (ended)
        return;
    processEvents();
    reportRequests();
    assignRound();
    advanceRobots();
    finishStep();
}

void Simulation::runAll() {
    while (!ended)
        step();
}

void Simulation::processEvents() {
    while (nextEvent < events.size() && events[nextEvent].time == now) {
        applyEvent(events[nextEvent]);
        nextEvent++;
    }
}

// Robot i gives its request back to the pending queue (offline, or no route left).
void Simulation::releaseRequestOf(int robotIdx, string why) {
    Robot& b = robots[robotIdx];
    int q = b.request;
    if (q < 0)
        return;
    b.request = -1;
    b.workLeft = 0;
    b.expectRemain = -1;
    requests[q].assignedRobot = -1;
    if (setRequestState(q, RQ_PENDING, why)) {
        stats.reassignments++;
        pending.push(q);
    }
}

void Simulation::applyEvent(const Event& e) {
    if (e.type == EV_BLOCK || e.type == EV_UNBLOCK) {
        Connection& c = env.conns[env.connIndex(e.a)];
        bool wantBlocked = (e.type == EV_BLOCK);
        string wantName = "OPEN";
        string oldName = "BLOCKED";
        string action = "UNBLOCK";
        if (wantBlocked) {
            wantName = "BLOCKED";
            oldName = "OPEN";
            action = "BLOCK";
        }

        if (c.blocked == wantBlocked) {
            logLine("REJECT", "event", c.id, wantName, wantName,
                action + " connection " + to_string(c.id) + ": nothing to change");
            stats.eventsRejected++;
        } else {
            c.blocked = wantBlocked;
            env.mapVersion++;
            logLine("EVENT", "connection", c.id, oldName, wantName,
                "between " + to_string(c.aId) + " and " + to_string(c.bId) + ", cost " + to_string(c.cost));
            stats.eventsApplied++;
        }
    }
    else if (e.type == EV_STATION_DOWN || e.type == EV_STATION_UP) {
        int li = env.locIndex(e.a);
        Location& l = env.locs[li];
        bool wantUp = (e.type == EV_STATION_UP);
        string wantName = "DOWN";
        string oldName = "UP";
        string action = "STATION_DOWN";
        if (wantUp) {
            wantName = "UP";
            oldName = "DOWN";
            action = "STATION_UP";
        }

        if (l.stationUp == wantUp) {
            logLine("REJECT", "event", l.id, wantName, wantName,
                action + " station " + to_string(l.id) + ": nothing to change");
            stats.eventsRejected++;
        } else {
            l.stationUp = wantUp;
            env.stationVersion++;
            logLine("EVENT", "station", l.id, oldName, wantName, l.name);
            stats.eventsApplied++;
            if (!wantUp) {
                for (int i = 0; i < robots.size(); i++)
                    if (robots[i].state == RS_RECHARGING && robots[i].at == li && robots[i].online)
                        setRobotState(i, RS_AVAILABLE, "station went down");
            }
        }
    }
    else if (e.type == EV_SET_URGENCY) {
        int q = requestMap.find(e.a);
        Request& r = requests[q];
        if (r.state == RQ_COMPLETED || r.state == RQ_FAILED) {
            logLine("REJECT", "event", r.id, reqStateName(r.state), reqStateName(r.state),
                "SET_URGENCY on a request that is already finished");
            stats.eventsRejected++;
        } else {
            logLine("EVENT", "request", r.id, "-", "-",
                "urgency " + to_string(r.urgency) + " -> " + to_string(e.b));
            r.urgency = e.b;
            pending.update(q);
            stats.eventsApplied++;
        }
    }
    else if (e.type == EV_ROBOT_OFFLINE || e.type == EV_ROBOT_ONLINE) {
        int i = robotMap.find(e.a);
        Robot& b = robots[i];
        bool wantOnline = (e.type == EV_ROBOT_ONLINE);
        string wantName = "OFFLINE";
        string action = "ROBOT_OFFLINE";
        if (wantOnline) {
            wantName = "ONLINE";
            action = "ROBOT_ONLINE";
        }

        if (b.online == wantOnline) {
            logLine("REJECT", "event", b.id, wantName, wantName,
                action + " robot " + to_string(b.id) + ": nothing to change");
            stats.eventsRejected++;
        } else {
            stats.eventsApplied++;
            if (wantOnline) {
                b.online = true;
                setRobotState(i, RS_AVAILABLE, "robot back online");
            } else {
                releaseRequestOf(i, "robot went offline");
                b.online = false;
                setRobotState(i, RS_UNAVAILABLE, "robot went offline");
            }
        }
    }
}

// A request is hopeless when no robot could ever finish it: nobody has the capability,
// the work alone is more than the robot's full battery, or the robot starts in a part of
// the map that is not connected to the request even with every road open.
bool Simulation::canEverBeServed(const Request& r) const {
    for (int i = 0; i < robots.size(); i++) {
        const Robot& b = robots[i];
        if (b.capability != CAT_GENERAL && b.capability != r.category)
            continue;
        if (r.duration * params.batteryPerWork > b.maxBattery)
            continue;
        if (comp[b.at] != comp[r.place])
            continue;
        return true;
    }
    return false;
}

void Simulation::reportRequests() {
    while (nextRequest < requests.size() && requests[nextRequest].reportTime == now) {
        int q = nextRequest;
        nextRequest++;
        Request& r = requests[q];
        stats.requestsReported++;
        setRequestState(q, RQ_PENDING, "reported");
        logLine("EVENT", "request", r.id, "-", "-",
            "loc=" + to_string(r.location) + " cat=" + categoryName(r.category) +
            " urgency=" + to_string(r.urgency) + " duration=" + to_string(r.duration));
        if (canEverBeServed(r)) {
            pending.push(q);
        } else {
            setRequestState(q, RQ_FAILED, "no robot can ever serve it");
            doneCount++;
        }
    }
}

void Simulation::finishStep() {
    for (int i = 0; i < robots.size(); i++)
        if (robots[i].online)
            robots[i].onlineSteps++;

    if (doneCount == requests.size() && nextEvent >= eventsDue) {
        endStep = now;
        writeEnd();
        return;
    }
    now++;
    if (now >= params.maxSteps) {
        endStep = params.maxSteps;
        writeEnd();
    }
}

void Simulation::writeEnd() {
    if (ended)
        return;
    ended = true;
    logLine("EVENT", "simulation", 0, "RUNNING", "ENDED", "end_step=" + to_string(endStep));
    logFile.flush();
}
