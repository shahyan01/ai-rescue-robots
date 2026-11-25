#include "Simulation.h"
#include "Utils.h"

Simulation::Simulation() {
	startClock = 0;
	ended = false;
	component = 0;
	eventsDue = 0;
	doneCount = 0;
	scenarioName[0] = '\0';
	params.maxSteps = 0;
	params.speed = 1;
	params.batteryPerCost = 1;
	params.batteryPerWork = 2;
	params.rechargePerStep = 5;
	params.lowBatteryPercent = 30;
	robots = 0;
	numRobots = 0;
	requests = 0;
	numRequests = 0;
	events = 0;
	numEvents = 0;
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
	if (logFile.is_open())
		logFile.close();
	delete[] component;
	delete[] robots;
	delete[] requests;
	delete[] events;
}

void Simulation::reserveRobots(int n) {
	delete[] robots;
	robots = new Robot[n + 1];
	numRobots = 0;
}

void Simulation::reserveRequests(int n) {
	delete[] requests;
	requests = new Request[n + 1];
	numRequests = 0;
}

void Simulation::reserveEvents(int n) {
	delete[] events;
	events = new Event[n + 1];
	numEvents = 0;
}

int Simulation::findRobot(int id) {
	for (int i = 0; i < numRobots; i++)
		if (robots[i].id == id)
			return i;
	return -1;
}

int Simulation::findRequest(int id) {
	for (int i = 0; i < numRequests; i++)
		if (requests[i].id == id)
			return i;
	return -1;
}

void Simulation::prepare() {
	int n = env.numLocs;

	// Label the connected parts of the map, treating every road as open.
	// A request is hopeless if no robot starts in the same part as the request.
	component = new int[n + 1];
	int* stack = new int[n + 1];
	for (int i = 0; i < n; i++)
		component[i] = -1;
	int label = 0;
	for (int s = 0; s < n; s++) {
		if (component[s] != -1)
			continue;
		component[s] = label;
		int top = 0;
		stack[top] = s;
		top++;
		while (top > 0) {
			top--;
			int u = stack[top];
			for (AdjNode* r = env.locs[u].firstRoad; r != 0; r = r->next) {
				int v = r->to;
				if (component[v] == -1) {
					component[v] = label;
					stack[top] = v;
					top++;
				}
			}
		}
		label++;
	}
	delete[] stack;

	for (int i = 0; i < numRobots; i++) {
		Robot& b = robots[i];
		b.at = env.findLocation(b.location);
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
	for (int i = 0; i < numRequests; i++) {
		Request& r = requests[i];
		r.place = env.findLocation(r.location);
		r.state = RQ_NOT_REPORTED;
		r.assignedRobot = -1;
		r.heapPos = -1;
		r.arriveTime = -1;
		r.doneTime = -1;
	}
	pending.init(requests, numRequests);

	eventsDue = 0;
	for (int i = 0; i < numEvents; i++)
		if (events[i].time < params.maxSteps)
			eventsDue++;

	startClock = clock();
}

bool Simulation::openLog(const char* path) {
	logFile.open(path);
	if (!logFile.is_open())
		return false;
	logFile << "step,kind,subject,id,from,to,detail" << endl;
	logStart("EVENT", "simulation", 0, "-", "RUNNING");
	logFile << "scenario=" << scenarioName << " max_steps=" << params.maxSteps
		<< " locations=" << env.numLocs << " connections=" << env.numConns
		<< " robots=" << numRobots << " requests=" << numRequests
		<< " events=" << numEvents << endl;
	return true;
}

// Writes the first six columns of a log line. The caller then writes the detail text.
void Simulation::logStart(const char* kind, const char* subject, int id, const char* from, const char* to) {
	logFile << now << "," << kind << "," << subject << "," << id << "," << from << "," << to << ",";
}

// Only the transitions of the request state machine are allowed. Anything else is rejected.
bool Simulation::setRequestState(int q, int to, const char* detail) {
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
		logStart("REJECT", "request", r.id, requestStateName(from), requestStateName(to));
		logFile << "invalid transition" << endl;
		stats.badTransitions++;
		return false;
	}
	r.state = to;
	logStart("STATE", "request", r.id, requestStateName(from), requestStateName(to));
	logFile << detail << endl;
	return true;
}

bool Simulation::setRobotState(int i, int to, const char* detail) {
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
		logStart("REJECT", "robot", b.id, robotStateName(from), robotStateName(to));
		logFile << "invalid transition" << endl;
		stats.badTransitions++;
		return false;
	}
	b.state = to;
	b.warned = false;
	logStart("STATE", "robot", b.id, robotStateName(from), robotStateName(to));
	logFile << detail << endl;
	return true;
}

bool Simulation::isFinished() {
	return ended;
}

// One time step. The order never changes:
//   1. events due at this step, in file order
//   2. requests reported at this step, in file order
//   3. assignment round
//   4. robots move, work or recharge
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
	while (nextEvent < numEvents && events[nextEvent].time == now) {
		applyEvent(events[nextEvent]);
		nextEvent++;
	}
}

// A robot gives its request back to the pending queue (it went offline, or has no route left).
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

void Simulation::applyEvent(Event& e) {
	if (e.type == EV_BLOCK || e.type == EV_UNBLOCK) {
		Connection& c = env.conns[env.findConnection(e.a)];
		bool wantBlocked = (e.type == EV_BLOCK);
		if (c.blocked == wantBlocked) {
			logStart("REJECT", "event", c.id, wantBlocked ? "BLOCKED" : "OPEN", wantBlocked ? "BLOCKED" : "OPEN");
			logFile << (wantBlocked ? "BLOCK" : "UNBLOCK") << " connection " << c.id << ": nothing to change" << endl;
			stats.eventsRejected++;
		} else {
			c.blocked = wantBlocked;
			env.mapVersion++;
			logStart("EVENT", "connection", c.id, wantBlocked ? "OPEN" : "BLOCKED", wantBlocked ? "BLOCKED" : "OPEN");
			logFile << "between " << c.aId << " and " << c.bId << ", cost " << c.cost << endl;
			stats.eventsApplied++;
		}
	} else if (e.type == EV_STATION_DOWN || e.type == EV_STATION_UP) {
		int li = env.findLocation(e.a);
		Location& l = env.locs[li];
		bool wantUp = (e.type == EV_STATION_UP);
		if (l.stationUp == wantUp) {
			logStart("REJECT", "event", l.id, wantUp ? "UP" : "DOWN", wantUp ? "UP" : "DOWN");
			logFile << (wantUp ? "STATION_UP" : "STATION_DOWN") << " station " << l.id << ": nothing to change" << endl;
			stats.eventsRejected++;
		} else {
			l.stationUp = wantUp;
			env.stationVersion++;
			logStart("EVENT", "station", l.id, wantUp ? "DOWN" : "UP", wantUp ? "UP" : "DOWN");
			logFile << l.name << endl;
			stats.eventsApplied++;
			if (!wantUp) {
				for (int i = 0; i < numRobots; i++)
					if (robots[i].state == RS_RECHARGING && robots[i].at == li && robots[i].online)
						setRobotState(i, RS_AVAILABLE, "station went down");
			}
		}
	} else if (e.type == EV_SET_URGENCY) {
		int q = findRequest(e.a);
		Request& r = requests[q];
		if (r.state == RQ_COMPLETED || r.state == RQ_FAILED) {
			logStart("REJECT", "event", r.id, requestStateName(r.state), requestStateName(r.state));
			logFile << "SET_URGENCY on a request that is already finished" << endl;
			stats.eventsRejected++;
		} else {
			logStart("EVENT", "request", r.id, "-", "-");
			logFile << "urgency " << r.urgency << " -> " << e.b << endl;
			r.urgency = e.b;
			pending.update(q);
			stats.eventsApplied++;
		}
	} else {
		int i = findRobot(e.a);
		Robot& b = robots[i];
		bool wantOnline = (e.type == EV_ROBOT_ONLINE);
		if (b.online == wantOnline) {
			logStart("REJECT", "event", b.id, wantOnline ? "ONLINE" : "OFFLINE", wantOnline ? "ONLINE" : "OFFLINE");
			logFile << (wantOnline ? "ROBOT_ONLINE" : "ROBOT_OFFLINE") << " robot " << b.id << ": nothing to change" << endl;
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
// the work alone needs more than the robot's full battery, or the robot starts in a part
// of the map that is not connected to the request even with every road open.
bool Simulation::canEverBeServed(Request& r) {
	for (int i = 0; i < numRobots; i++) {
		Robot& b = robots[i];
		if (b.capability != CAT_GENERAL && b.capability != r.category)
			continue;
		if (r.duration * params.batteryPerWork > b.maxBattery)
			continue;
		if (component[b.at] != component[r.place])
			continue;
		return true;
	}
	return false;
}

void Simulation::reportRequests() {
	while (nextRequest < numRequests && requests[nextRequest].reportTime == now) {
		int q = nextRequest;
		nextRequest++;
		Request& r = requests[q];
		stats.requestsReported++;
		setRequestState(q, RQ_PENDING, "reported");
		logStart("EVENT", "request", r.id, "-", "-");
		logFile << "loc=" << r.location << " cat=" << categoryName(r.category)
			<< " urgency=" << r.urgency << " duration=" << r.duration << endl;
		if (canEverBeServed(r)) {
			pending.push(q);
		} else {
			setRequestState(q, RQ_FAILED, "no robot can ever serve it");
			doneCount++;
		}
	}
}

void Simulation::finishStep() {
	for (int i = 0; i < numRobots; i++)
		if (robots[i].online)
			robots[i].onlineSteps++;

	if (doneCount == numRequests && nextEvent >= eventsDue) {
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
	logStart("EVENT", "simulation", 0, "RUNNING", "ENDED");
	logFile << "end_step=" << endStep << endl;
}
