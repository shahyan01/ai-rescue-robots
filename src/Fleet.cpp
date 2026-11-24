#include "Simulation.h"

void Simulation::spend(Robot& b, int amount) {
    int used = amount;
    if (b.battery < amount)
        used = b.battery;
    b.battery -= used;
    b.consumed += used;
}

bool Simulation::atWorkingStation(const Robot& b) const {
    const Location& l = env.locs[b.at];
    return l.type == LT_CHARGING && l.stationUp;
}

void Simulation::startRoad(Robot& b, int conn, int to) {
    const Connection& c = env.conns[conn];
    logLine("MOVE", "robot", b.id, "-", "-",
        "from=" + to_string(env.locs[b.at].id) + " to=" + to_string(env.locs[to].id) +
        " conn=" + to_string(c.id) + " cost=" + to_string(c.cost));
    b.roadTo = to;
    b.roadRemain = c.cost;
}

// One step along the current road: up to speed cost units, never more than what is left.
// Returns true when the robot reaches the end of the road in this step.
bool Simulation::moveAlongRoad(Robot& b) {
    int m = params.speed;
    if (b.roadRemain < m)
        m = b.roadRemain;
    b.roadRemain -= m;
    b.moved += m;
    spend(b, m * params.batteryPerCost);
    if (b.roadRemain > 0)
        return false;
    b.at = b.roadTo;
    b.roadTo = -1;
    return true;
}

void Simulation::advanceRobots() {
    for (int i = 0; i < robots.size(); i++) {
        Robot& b = robots[i];
        if (!b.online)
            continue;

        // A robot that came back online in the middle of a road finishes it first.
        if (b.roadTo != -1 && b.request == -1 && b.state == RS_AVAILABLE) {
            moveAlongRoad(b);
            continue;
        }

        if (b.state == RS_TRAVELLING)
            travel(i);
        else if (b.state == RS_PERFORMING)
            doWork(i);
        else if (b.state == RS_RETURNING)
            returnTrip(i);
        else if (b.state == RS_RECHARGING) {
            if (atWorkingStation(b))
                rechargeStep(i);
            else
                setRobotState(i, RS_AVAILABLE, "station not working");
        }
        else if (b.state == RS_AVAILABLE)
            idleStep(i);
    }
}

// Going to the request. At every location the next road comes from a fresh tree, so a
// road that closed ahead causes a detour and one that reopened is used at once.
void Simulation::travel(int i) {
    Robot& b = robots[i];
    Request& q = requests[b.request];

    if (b.roadTo == -1) {
        if (b.at == q.place) {
            arrive(i);
            return;
        }
        const Tree& t = env.treeFor(q.place);
        int d = t.dist[b.at];
        if (d >= INF_COST || b.battery < d * params.batteryPerCost) {
            logLine("WARN", "robot", b.id, "-", "-",
                "no usable route to request " + to_string(q.id) + " from location " +
                to_string(env.locs[b.at].id) + ", request goes back to pending");
            releaseRequestOf(i, "route lost");
            setRobotState(i, RS_AVAILABLE, "route lost");
            return;
        }
        if (b.expectRemain >= 0 && d > b.expectRemain)
            logLine("REROUTE", "robot", b.id, "-", "-",
                "request " + to_string(q.id) + ": route left grew from " + to_string(b.expectRemain) +
                " to " + to_string(d) + " at location " + to_string(env.locs[b.at].id));
        int next = t.parent[b.at];
        startRoad(b, t.pconn[b.at], next);
        b.expectRemain = t.dist[next];
    }

    b.travelSteps++;
    if (moveAlongRoad(b) && b.at == q.place)
        arrive(i);
}

// The robot is on the request location: work starts in the next step.
void Simulation::arrive(int i) {
    Robot& b = robots[i];
    Request& q = requests[b.request];
    setRequestState(b.request, RQ_IN_PROGRESS, "robot arrived");
    setRobotState(i, RS_PERFORMING, "working");
    b.workLeft = q.duration;
    b.expectRemain = -1;
    q.arriveTime = now;
    logLine("ARRIVE", "robot", b.id, "-", "-",
        "request=" + to_string(q.id) + " location=" + to_string(env.locs[b.at].id) +
        " response=" + to_string(now - q.reportTime));
}

void Simulation::doWork(int i) {
    Robot& b = robots[i];
    b.workLeft--;
    b.workSteps++;
    spend(b, params.batteryPerWork);
    if (b.workLeft > 0)
        return;

    int qi = b.request;
    Request& q = requests[qi];
    if (setRequestState(qi, RQ_COMPLETED, "work finished")) {
        q.doneTime = now;
        doneCount++;
    }
    logLine("WORK_DONE", "robot", b.id, "-", "-",
        "request=" + to_string(q.id) + " battery=" + to_string(b.battery));
    b.request = -1;
    setRobotState(i, RS_AVAILABLE, "request done");
}

void Simulation::rechargeStep(int i) {
    Robot& b = robots[i];
    int add = params.rechargePerStep;
    if (b.battery + add > b.maxBattery)
        add = b.maxBattery - b.battery;
    b.battery += add;
    b.recharged += add;
    if (b.battery >= b.maxBattery) {
        logLine("BATTERY", "robot", b.id, "-", "-", "fully charged");
        setRobotState(i, RS_AVAILABLE, "fully charged");
    }
}

// Idle robot with nothing to do: recharge if standing on a working station, or head
// for the nearest working station when the battery is below the low-battery level.
void Simulation::idleStep(int i) {
    Robot& b = robots[i];

    if (atWorkingStation(b)) {
        if (b.battery < b.maxBattery) {
            setRobotState(i, RS_RECHARGING, "on a working station");
            rechargeStep(i);
        }
        return;
    }

    if (b.battery * 100 >= params.lowBatteryPercent * b.maxBattery)
        return;

    const Tree& t = env.chargerTree();
    int d = t.dist[b.at];
    if (d >= INF_COST || b.battery < d * params.batteryPerCost) {
        if (!b.warned) {
            b.warned = true;
            logLine("BATTERY", "robot", b.id, "-", "-",
                "low battery (" + to_string(b.battery) + ") and no working station within reach, staying at location " +
                to_string(env.locs[b.at].id));
        }
        return;
    }
    logLine("BATTERY", "robot", b.id, "-", "-",
        "low battery (" + to_string(b.battery) + "), heading for the nearest working station, cost " + to_string(d));
    setRobotState(i, RS_RETURNING, "low battery");
    returnTrip(i);
}

// Walking to the nearest working station. Re-planned at every location, so a station
// that went down is dropped and another one is chosen.
void Simulation::returnTrip(int i) {
    Robot& b = robots[i];

    if (b.roadTo == -1) {
        if (atWorkingStation(b)) {
            setRobotState(i, RS_RECHARGING, "reached station");
            return;
        }
        const Tree& t = env.chargerTree();
        int d = t.dist[b.at];
        if (d >= INF_COST || b.battery < d * params.batteryPerCost) {
            logLine("BATTERY", "robot", b.id, "-", "-",
                "no working station within reach, staying at location " + to_string(env.locs[b.at].id));
            setRobotState(i, RS_AVAILABLE, "no station within reach");
            return;
        }
        startRoad(b, t.pconn[b.at], t.parent[b.at]);
    }

    if (moveAlongRoad(b) && atWorkingStation(b))
        setRobotState(i, RS_RECHARGING, "reached station");
}
