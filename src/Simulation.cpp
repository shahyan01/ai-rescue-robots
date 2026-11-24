#include "Simulation.h"
#include <cstdarg>
#include <cstring>

Simulation::Simulation() : logFile(0), startClock(0), ended(false), comp(0), eventsDue(0), doneCount(0) {
	scenarioName[0] = '\0';
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
}

Simulation::~Simulation() {
	if (logFile)
		fclose(logFile);
	delete[] comp;
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
		stack[top++] = s;
		while (top > 0) {
			int u = stack[--top];
			for (int e = env.locs[u].adjHead; e != -1; e = env.adj[e].next) {
				int v = env.adj[e].to;
				if (comp[v] == -1) {
					comp[v] = label;
					stack[top++] = v;
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

	startClock = clock();
}

bool Simulation::openLog(const char* path) {
	logFile = fopen(path, "w");
	if (!logFile)
		return false;
	fprintf(logFile, "step,kind,subject,id,from,to,detail\n");
	logLine("EVENT", "simulation", 0, "-", "RUNNING", "scenario=%s max_steps=%d locations=%d connections=%d robots=%d requests=%d events=%d",
		scenarioName, params.maxSteps, env.locs.size(), env.conns.size(),
		robots.size(), requests.size(), events.size());
	return true;
}

void Simulation::logLine(const char* kind, const char* subject, int id, const char* from, const char* to, const char* fmt, ...) {
	if (!logFile)
		return;
	fprintf(logFile, "%d,%s,%s,%d,%s,%s,", now, kind, subject, id, from, to);
	va_list ap;
	va_start(ap, fmt);
	vfprintf(logFile, fmt, ap);
	va_end(ap);
	fputc('\n', logFile);
}

// Only the transitions of the request state machine are allowed.
// Anything else is rejected and logged.
bool Simulation::setRequestState(int q, ReqState to, const char* detail) {
	Request& r = requests[q];
	ReqState from = r.state;
	bool ok = false;
	switch (from) {
	case RQ_NOT_REPORTED: ok = (to == RQ_PENDING); break;
	case RQ_PENDING: ok = (to == RQ_ASSIGNED || to == RQ_FAILED); break;
	case RQ_ASSIGNED: ok = (to == RQ_IN_PROGRESS || to == RQ_PENDING); break;
	case RQ_IN_PROGRESS: ok = (to == RQ_COMPLETED || to == RQ_PENDING); break;
	default: ok = false; break;
	}
	if (!ok) {
		logLine("REJECT", "request", r.id, reqStateName(from), reqStateName(to), "invalid transition");
		stats.badTransitions++;
		return false;
	}
	r.state = to;
	logLine("STATE", "request", r.id, reqStateName(from), reqStateName(to), "%s", detail);
	return true;
}

bool Simulation::setRobotState(int i, RobotState to, const char* detail) {
	Robot& b = robots[i];
	RobotState from = b.state;
	if (from == to)
		return true;
	bool ok = false;
	if (to == RS_UNAVAILABLE)
		ok = true;
	else {
		switch (from) {
		case RS_AVAILABLE: ok = (to == RS_TRAVELLING || to == RS_RETURNING || to == RS_RECHARGING); break;
		case RS_TRAVELLING: ok = (to == RS_PERFORMING || to == RS_AVAILABLE); break;
		case RS_PERFORMING: ok = (to == RS_AVAILABLE); break;
		case RS_RETURNING: ok = (to == RS_RECHARGING || to == RS_AVAILABLE); break;
		case RS_RECHARGING: ok = (to == RS_AVAILABLE || to == RS_TRAVELLING); break;
		case RS_UNAVAILABLE: ok = (to == RS_AVAILABLE); break;
		}
	}
	if (!ok) {
		logLine("REJECT", "robot", b.id, robotStateName(from), robotStateName(to), "invalid transition");
		stats.badTransitions++;
		return false;
	}
	b.state = to;
	b.warned = false;
	logLine("STATE", "robot", b.id, robotStateName(from), robotStateName(to), "%s", detail);
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
void Simulation::releaseRequestOf(int robotIdx, const char* why) {
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
	switch (e.type) {
	case EV_BLOCK:
	case EV_UNBLOCK: {
		Connection& c = env.conns[env.connIndex(e.a)];
		bool wantBlocked = (e.type == EV_BLOCK);
		if (c.blocked == wantBlocked) {
			logLine("REJECT", "event", c.id, wantBlocked ? "BLOCKED" : "OPEN", wantBlocked ? "BLOCKED" : "OPEN",
				"%s connection %d: nothing to change", wantBlocked ? "BLOCK" : "UNBLOCK", c.id);
			stats.eventsRejected++;
		} else {
			c.blocked = wantBlocked;
			env.mapVersion++;
			logLine("EVENT", "connection", c.id, wantBlocked ? "OPEN" : "BLOCKED", wantBlocked ? "BLOCKED" : "OPEN",
				"between %d and %d, cost %d", c.aId, c.bId, c.cost);
			stats.eventsApplied++;
		}
		break;
	}
	case EV_STATION_DOWN:
	case EV_STATION_UP: {
		int li = env.locIndex(e.a);
		Location& l = env.locs[li];
		bool wantUp = (e.type == EV_STATION_UP);
		if (l.stationUp == wantUp) {
			logLine("REJECT", "event", l.id, wantUp ? "UP" : "DOWN", wantUp ? "UP" : "DOWN",
				"%s station %d: nothing to change", wantUp ? "STATION_UP" : "STATION_DOWN", l.id);
			stats.eventsRejected++;
		} else {
			l.stationUp = wantUp;
			env.stationVersion++;
			logLine("EVENT", "station", l.id, wantUp ? "DOWN" : "UP", wantUp ? "UP" : "DOWN", "%s", l.name);
			stats.eventsApplied++;
			if (!wantUp) {
				for (int i = 0; i < robots.size(); i++)
					if (robots[i].state == RS_RECHARGING && robots[i].at == li && robots[i].online)
						setRobotState(i, RS_AVAILABLE, "station went down");
			}
		}
		break;
	}
	case EV_SET_URGENCY: {
		int q = requestMap.find(e.a);
		Request& r = requests[q];
		if (r.state == RQ_COMPLETED || r.state == RQ_FAILED) {
			logLine("REJECT", "event", r.id, reqStateName(r.state), reqStateName(r.state), "SET_URGENCY on a request that is already finished");
			stats.eventsRejected++;
		} else {
			logLine("EVENT", "request", r.id, "-", "-", "urgency %d -> %d", r.urgency, e.b);
			r.urgency = e.b;
			pending.update(q);
			stats.eventsApplied++;
		}
		break;
	}
	case EV_ROBOT_OFFLINE:
	case EV_ROBOT_ONLINE: {
		int i = robotMap.find(e.a);
		Robot& b = robots[i];
		bool wantOnline = (e.type == EV_ROBOT_ONLINE);
		if (b.online == wantOnline) {
			logLine("REJECT", "event", b.id, wantOnline ? "ONLINE" : "OFFLINE", wantOnline ? "ONLINE" : "OFFLINE",
				"%s robot %d: nothing to change", wantOnline ? "ROBOT_ONLINE" : "ROBOT_OFFLINE", b.id);
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
		break;
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
		int q = nextRequest++;
		Request& r = requests[q];
		stats.requestsReported++;
		setRequestState(q, RQ_PENDING, "reported");
		logLine("EVENT", "request", r.id, "-", "-", "loc=%d cat=%s urgency=%d duration=%d",
			r.location, categoryName(r.category), r.urgency, r.duration);
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
	logLine("EVENT", "simulation", 0, "RUNNING", "ENDED", "end_step=%d", endStep);
	if (logFile)
		fflush(logFile);
}
