#include "Loader.h"
#include "Utils.h"

const int SEC_NONE = 0;
const int SEC_LOC = 1;
const int SEC_CONN = 2;
const int SEC_ROB = 3;
const int SEC_REQ = 4;
const int SEC_EVT = 5;
const int SEC_SKIP = 6;

const int MAX_WORDS = 16;
const int MAX_LINE = 512;

const int LIMIT_ID = 1000000;

Loader::Loader() {
	errorCount = 0;
	warningCount = 0;
	haveMaxSteps = false;
}

void Loader::warn(const char* text) {
	cout << "  warning: " << text << endl;
	warningCount++;
}

const char* sectionName(int section) {
	if (section == SEC_LOC) return "LOCATIONS";
	if (section == SEC_CONN) return "CONNECTIONS";
	if (section == SEC_ROB) return "ROBOTS";
	if (section == SEC_REQ) return "REQUESTS";
	return "EVENTS";
}

bool isKeyword(const char* w) {
	return isSame(w, "SCENARIO") || isSame(w, "PARAM") || isSame(w, "LOCATIONS") ||
		isSame(w, "CONNECTIONS") || isSame(w, "ROBOTS") || isSame(w, "REQUESTS") ||
		isSame(w, "EVENTS") || isSame(w, "END");
}

void Loader::parseParam(char* words[], int n, int lineNo, Simulation& sim) {
	int v;
	if (n != 3 || !toInt(words[2], v)) {
		cout << "  - line " << lineNo << ": PARAM needs a key and a whole-number value" << endl;
		errorCount++;
		return;
	}
	if (isSame(words[1], "max_steps")) {
		if (v < 1) { cout << "  - line " << lineNo << ": max_steps must be at least 1" << endl; errorCount++; return; }
		sim.params.maxSteps = v;
		haveMaxSteps = true;
	} else if (isSame(words[1], "speed")) {
		if (v < 1) { cout << "  - line " << lineNo << ": speed must be at least 1" << endl; errorCount++; return; }
		sim.params.speed = v;
	} else if (isSame(words[1], "battery_per_cost_unit")) {
		if (v < 0) { cout << "  - line " << lineNo << ": battery_per_cost_unit cannot be negative" << endl; errorCount++; return; }
		sim.params.batteryPerCost = v;
	} else if (isSame(words[1], "battery_per_work_step")) {
		if (v < 0) { cout << "  - line " << lineNo << ": battery_per_work_step cannot be negative" << endl; errorCount++; return; }
		sim.params.batteryPerWork = v;
	} else if (isSame(words[1], "recharge_per_step")) {
		if (v < 0) { cout << "  - line " << lineNo << ": recharge_per_step cannot be negative" << endl; errorCount++; return; }
		sim.params.rechargePerStep = v;
	} else if (isSame(words[1], "low_battery_percent")) {
		if (v < 0 || v > 100) { cout << "  - line " << lineNo << ": low_battery_percent must be 0 to 100" << endl; errorCount++; return; }
		sim.params.lowBatteryPercent = v;
	}
	// other keys are ignored, so extended files still load
}

void Loader::parseLocation(char* words[], int n, int lineNo, Simulation& sim) {
	Location l;
	if (n != 3) {
		cout << "  - line " << lineNo << ": location needs 3 fields (id name type), found " << n << endl;
		errorCount++;
		return;
	}
	if (!toInt(words[0], l.id)) {
		cout << "  - line " << lineNo << ": location id '" << words[0] << "' is not a number" << endl;
		errorCount++;
		return;
	}
	if (textLength(words[1]) >= 32) {
		cout << "  - line " << lineNo << ": location name '" << words[1] << "' is too long (max 31 characters)" << endl;
		errorCount++;
		return;
	}
	copyText(l.name, words[1], 32);
	l.type = locationTypeFromWord(words[2]);
	if (l.type == -1) {
		cout << "  - line " << lineNo << ": unknown location type '" << words[2] << "'" << endl;
		errorCount++;
		return;
	}
	l.stationUp = true;
	l.firstRoad = 0;
	sim.env.locs[sim.env.numLocs] = l;
	sim.env.numLocs++;
}

void Loader::parseConnection(char* words[], int n, int lineNo, Simulation& sim) {
	Connection c;
	if (n != 5) {
		cout << "  - line " << lineNo << ": connection needs 5 fields (id a b cost status), found " << n << endl;
		errorCount++;
		return;
	}
	if (!toInt(words[0], c.id) || !toInt(words[1], c.aId) || !toInt(words[2], c.bId) || !toInt(words[3], c.cost)) {
		cout << "  - line " << lineNo << ": connection id, endpoints and cost must be whole numbers" << endl;
		errorCount++;
		return;
	}
	if (c.cost < 1) {
		cout << "  - line " << lineNo << ": connection " << c.id << " has cost " << c.cost << ", cost must be at least 1" << endl;
		errorCount++;
		return;
	}
	if (isSame(words[4], "OPEN"))
		c.blocked = false;
	else if (isSame(words[4], "BLOCKED"))
		c.blocked = true;
	else {
		cout << "  - line " << lineNo << ": connection status '" << words[4] << "' must be OPEN or BLOCKED" << endl;
		errorCount++;
		return;
	}
	c.aIdx = -1;
	c.bIdx = -1;
	sim.env.conns[sim.env.numConns] = c;
	sim.env.numConns++;
}

void Loader::parseRobot(char* words[], int n, int lineNo, Simulation& sim) {
	Robot r;
	if (n != 5) {
		cout << "  - line " << lineNo << ": robot needs 5 fields (id start capability max initial), found " << n << endl;
		errorCount++;
		return;
	}
	if (!toInt(words[0], r.id) || !toInt(words[1], r.location) || !toInt(words[3], r.maxBattery) || !toInt(words[4], r.battery)) {
		cout << "  - line " << lineNo << ": robot id, start and battery values must be whole numbers" << endl;
		errorCount++;
		return;
	}
	r.capability = categoryFromWord(words[2]);
	if (r.capability == -1) {
		cout << "  - line " << lineNo << ": unknown robot capability '" << words[2] << "'" << endl;
		errorCount++;
		return;
	}
	if (r.maxBattery < 1 || r.battery < 0 || r.battery > r.maxBattery) {
		cout << "  - line " << lineNo << ": robot " << r.id << " has battery " << r.battery << " with max " << r.maxBattery
			<< " (need 0 <= initial <= max, max >= 1)" << endl;
		errorCount++;
		return;
	}
	r.state = RS_AVAILABLE;
	r.online = true;
	sim.robots[sim.numRobots] = r;
	sim.numRobots++;
}

void Loader::parseRequest(char* words[], int n, int lineNo, Simulation& sim) {
	Request r;
	if (n != 6) {
		cout << "  - line " << lineNo << ": request needs 6 fields (id time location category urgency duration), found " << n << endl;
		errorCount++;
		return;
	}
	if (!toInt(words[0], r.id) || !toInt(words[1], r.reportTime) || !toInt(words[2], r.location) ||
		!toInt(words[4], r.urgency) || !toInt(words[5], r.duration)) {
		cout << "  - line " << lineNo << ": request numeric fields must be whole numbers" << endl;
		errorCount++;
		return;
	}
	r.category = categoryFromWord(words[3]);
	if (r.category == -1 || r.category == CAT_GENERAL) {
		cout << "  - line " << lineNo << ": unknown request category '" << words[3] << "'" << endl;
		errorCount++;
		return;
	}
	if (r.reportTime < 0 || r.urgency < 1 || r.urgency > 5 || r.duration < 1) {
		cout << "  - line " << lineNo << ": request " << r.id << " needs time >= 0, urgency 1-5 and duration >= 1" << endl;
		errorCount++;
		return;
	}
	r.state = RQ_NOT_REPORTED;
	r.assignedRobot = -1;
	sim.requests[sim.numRequests] = r;
	sim.numRequests++;
}

void Loader::parseEvent(char* words[], int n, int lineNo, Simulation& sim) {
	Event e;
	e.b = 0;
	if (n < 3) {
		cout << "  - line " << lineNo << ": event needs at least 3 fields (time TYPE target)" << endl;
		errorCount++;
		return;
	}
	if (!toInt(words[0], e.time) || e.time < 0) {
		cout << "  - line " << lineNo << ": event time '" << words[0] << "' must be a whole number >= 0" << endl;
		errorCount++;
		return;
	}
	int expected = 3;
	if (isSame(words[1], "BLOCK")) e.type = EV_BLOCK;
	else if (isSame(words[1], "UNBLOCK")) e.type = EV_UNBLOCK;
	else if (isSame(words[1], "STATION_DOWN")) e.type = EV_STATION_DOWN;
	else if (isSame(words[1], "STATION_UP")) e.type = EV_STATION_UP;
	else if (isSame(words[1], "ROBOT_OFFLINE")) e.type = EV_ROBOT_OFFLINE;
	else if (isSame(words[1], "ROBOT_ONLINE")) e.type = EV_ROBOT_ONLINE;
	else if (isSame(words[1], "SET_URGENCY")) { e.type = EV_SET_URGENCY; expected = 4; }
	else {
		cout << "  - line " << lineNo << ": unknown event type '" << words[1] << "'" << endl;
		errorCount++;
		return;
	}
	if (n != expected) {
		cout << "  - line " << lineNo << ": " << words[1] << " needs " << expected << " fields, found " << n << endl;
		errorCount++;
		return;
	}
	if (!toInt(words[2], e.a) || (expected == 4 && !toInt(words[3], e.b))) {
		cout << "  - line " << lineNo << ": event target must be a whole number" << endl;
		errorCount++;
		return;
	}
	if (e.type == EV_SET_URGENCY && (e.b < 1 || e.b > 5)) {
		cout << "  - line " << lineNo << ": new urgency " << e.b << " must be 1-5" << endl;
		errorCount++;
		return;
	}
	sim.events[sim.numEvents] = e;
	sim.numEvents++;
}

void Loader::closeSection(int section, int remaining, int declared, int lineNo) {
	if (section != SEC_NONE && section != SEC_SKIP && remaining > 0) {
		cout << "  - line " << lineNo << ": " << sectionName(section) << " declares " << declared
			<< " records but only " << (declared - remaining) << " were found" << endl;
		errorCount++;
	}
}

// Insertion sort by time. Records with the same time keep their order in the file.
void Loader::sortEvents(Simulation& sim) {
	for (int i = 1; i < sim.numEvents; i++) {
		Event key = sim.events[i];
		int j = i - 1;
		while (j >= 0 && sim.events[j].time > key.time) {
			sim.events[j + 1] = sim.events[j];
			j--;
		}
		sim.events[j + 1] = key;
	}
}

void Loader::sortRequests(Simulation& sim) {
	for (int i = 1; i < sim.numRequests; i++) {
		Request key = sim.requests[i];
		int j = i - 1;
		while (j >= 0 && sim.requests[j].reportTime > key.reportTime) {
			sim.requests[j + 1] = sim.requests[j];
			j--;
		}
		sim.requests[j + 1] = key;
	}
}

// Looks for an id that is negative, too big, or used twice. Returns true if one is found.
bool findBadId(int ids[], int n, int& badId) {
	for (int i = 0; i < n; i++) {
		if (ids[i] < 0 || ids[i] > LIMIT_ID) {
			badId = ids[i];
			return true;
		}
		for (int j = 0; j < i; j++)
			if (ids[j] == ids[i]) {
				badId = ids[i];
				return true;
			}
	}
	return false;
}

// Checks that ids are unique and that everything refers to something that exists.
void Loader::validate(Simulation& sim) {
	Environment& env = sim.env;
	int badId = 0;

	int* ids = new int[env.numLocs + env.numConns + sim.numRobots + sim.numRequests + 1];

	for (int i = 0; i < env.numLocs; i++)
		ids[i] = env.locs[i].id;
	bool locOk = !findBadId(ids, env.numLocs, badId);
	if (!locOk) {
		cout << "  - location id " << badId << " is duplicated or out of range (0 to " << LIMIT_ID << ")" << endl;
		errorCount++;
	}

	for (int i = 0; i < env.numConns; i++)
		ids[i] = env.conns[i].id;
	bool connOk = !findBadId(ids, env.numConns, badId);
	if (!connOk) {
		cout << "  - connection id " << badId << " is duplicated or out of range (0 to " << LIMIT_ID << ")" << endl;
		errorCount++;
	}

	for (int i = 0; i < sim.numRobots; i++)
		ids[i] = sim.robots[i].id;
	if (findBadId(ids, sim.numRobots, badId)) {
		cout << "  - robot id " << badId << " is duplicated or out of range" << endl;
		errorCount++;
	}

	sortRequests(sim);
	for (int i = 0; i < sim.numRequests; i++)
		ids[i] = sim.requests[i].id;
	if (findBadId(ids, sim.numRequests, badId)) {
		cout << "  - request id " << badId << " is duplicated or out of range" << endl;
		errorCount++;
	}
	delete[] ids;

	if (locOk) {
		for (int i = 0; i < env.numConns; i++) {
			Connection& c = env.conns[i];
			if (env.findLocation(c.aId) == -1 || env.findLocation(c.bId) == -1) {
				cout << "  - connection " << c.id << " refers to a location that does not exist (" << c.aId << ", " << c.bId << ")" << endl;
				errorCount++;
			} else if (c.aId == c.bId) {
				cout << "  - connection " << c.id << " connects location " << c.aId << " to itself" << endl;
				errorCount++;
			}
		}
		for (int i = 0; i < sim.numRobots; i++)
			if (env.findLocation(sim.robots[i].location) == -1) {
				cout << "  - robot " << sim.robots[i].id << " starts at location " << sim.robots[i].location << " which does not exist" << endl;
				errorCount++;
			}
		for (int i = 0; i < sim.numRequests; i++)
			if (env.findLocation(sim.requests[i].location) == -1) {
				cout << "  - request " << sim.requests[i].id << " is at location " << sim.requests[i].location << " which does not exist" << endl;
				errorCount++;
			}
	}

	sortEvents(sim);
	for (int i = 0; i < sim.numEvents; i++) {
		Event& e = sim.events[i];
		if (e.type == EV_BLOCK || e.type == EV_UNBLOCK) {
			if (connOk && env.findConnection(e.a) == -1) {
				cout << "  - event at time " << e.time << " refers to unknown connection " << e.a << endl;
				errorCount++;
			}
		} else if (e.type == EV_STATION_DOWN || e.type == EV_STATION_UP) {
			if (locOk) {
				int li = env.findLocation(e.a);
				if (li == -1) {
					cout << "  - event at time " << e.time << " refers to unknown location " << e.a << endl;
					errorCount++;
				} else if (env.locs[li].type != LT_CHARGING) {
					cout << "  - event at time " << e.time << ": location " << e.a << " is not a charging station" << endl;
					errorCount++;
				}
			}
		} else if (e.type == EV_SET_URGENCY) {
			if (sim.findRequest(e.a) == -1) {
				cout << "  - event at time " << e.time << " refers to unknown request " << e.a << endl;
				errorCount++;
			}
		} else {
			if (sim.findRobot(e.a) == -1) {
				cout << "  - event at time " << e.time << " refers to unknown robot " << e.a << endl;
				errorCount++;
			}
		}
	}

	// Warnings: legal, but they will never have an effect.
	int* urgency = new int[sim.numRequests + 1];
	for (int i = 0; i < sim.numRequests; i++)
		urgency[i] = sim.requests[i].urgency;
	for (int i = 0; i < sim.numEvents; i++) {
		Event& e = sim.events[i];
		if (e.time >= sim.params.maxSteps) {
			cout << "  warning: event " << (i + 1) << " at time " << e.time << " is after max_steps "
				<< sim.params.maxSteps << " and will never happen" << endl;
			warningCount++;
		}
		if (e.type == EV_SET_URGENCY) {
			int q = sim.findRequest(e.a);
			if (q != -1) {
				if (urgency[q] == e.b) {
					cout << "  warning: event at time " << e.time << " sets request " << e.a
						<< " to urgency " << e.b << ", which it already has" << endl;
					warningCount++;
				}
				urgency[q] = e.b;
			}
		}
	}
	for (int i = 0; i < sim.numRequests; i++)
		if (sim.requests[i].reportTime >= sim.params.maxSteps) {
			cout << "  warning: request " << sim.requests[i].id << " is reported at time " << sim.requests[i].reportTime
				<< ", after max_steps " << sim.params.maxSteps << ", and will never be reported" << endl;
			warningCount++;
		}
	delete[] urgency;
}

bool Loader::load(const char* path, Simulation& sim) {
	ifstream file(path);
	if (!file.is_open()) {
		cout << "  - cannot open file '" << path << "'" << endl;
		errorCount++;
		return false;
	}

	char line[MAX_LINE];
	char* words[MAX_WORDS];
	int lineNo = 0;
	int section = SEC_NONE;
	int remaining = 0;
	int declared = 0;
	bool seen[7] = {false, false, false, false, false, false, false};
	bool sawEnd = false;

	// The loader needs the arrays to exist even if a section is missing.
	sim.env.reserveLocations(0);
	sim.env.reserveConnections(0);
	sim.reserveRobots(0);
	sim.reserveRequests(0);
	sim.reserveEvents(0);

	while (true) {
		file.getline(line, MAX_LINE);
		if (file.eof() && line[0] == '\0')
			break;
		if (file.fail() && !file.eof()) {
			lineNo++;
			cout << "  - line " << lineNo << ": line is too long" << endl;
			errorCount++;
			file.clear();
			file.ignore(100000, '\n');
			continue;
		}
		lineNo++;

		int p = 0;
		while (line[p] == ' ' || line[p] == '\t')
			p++;
		if (line[p] == '#' || line[p] == '\n' || line[p] == '\r' || line[p] == '\0') {
			if (file.eof())
				break;
			continue;
		}

		if (sawEnd) {
			cout << "  - line " << lineNo << ": text after END" << endl;
			errorCount++;
			break;
		}

		int n = splitLine(line, words, MAX_WORDS);
		if (n == 0) {
			if (file.eof())
				break;
			continue;
		}

		if (isKeyword(words[0])) {
			closeSection(section, remaining, declared, lineNo);
			section = SEC_NONE;
			remaining = 0;

			if (isSame(words[0], "END")) {
				sawEnd = true;
			} else if (isSame(words[0], "SCENARIO")) {
				if (n >= 2)
					copyText(sim.scenarioName, words[1], 64);
			} else if (isSame(words[0], "PARAM")) {
				parseParam(words, n, lineNo, sim);
			} else {
				int s = SEC_EVT;
				if (isSame(words[0], "LOCATIONS")) s = SEC_LOC;
				else if (isSame(words[0], "CONNECTIONS")) s = SEC_CONN;
				else if (isSame(words[0], "ROBOTS")) s = SEC_ROB;
				else if (isSame(words[0], "REQUESTS")) s = SEC_REQ;

				int howMany;
				if (n != 2 || !toInt(words[1], howMany) || howMany < 0) {
					cout << "  - line " << lineNo << ": " << words[0] << " header needs a count (e.g. " << words[0] << " 10)" << endl;
					errorCount++;
				} else if (seen[s]) {
					cout << "  - line " << lineNo << ": section " << words[0] << " appears twice" << endl;
					errorCount++;
					section = SEC_SKIP;
					remaining = howMany;
				} else {
					seen[s] = true;
					section = s;
					declared = howMany;
					remaining = howMany;
					if (s == SEC_LOC) sim.env.reserveLocations(howMany);
					else if (s == SEC_CONN) sim.env.reserveConnections(howMany);
					else if (s == SEC_ROB) sim.reserveRobots(howMany);
					else if (s == SEC_REQ) sim.reserveRequests(howMany);
					else sim.reserveEvents(howMany);
				}
			}
			if (file.eof())
				break;
			continue;
		}

		// A line like "WEATHER 1" where a section header should be: unknown section.
		int howMany2;
		bool upper = true;
		for (int c = 0; words[0][c] != '\0'; c++)
			if (words[0][c] < 'A' || words[0][c] > 'Z')
				upper = false;
		if ((section == SEC_NONE || remaining == 0) && n == 2 && upper && toInt(words[1], howMany2) && howMany2 >= 0) {
			cout << "  - line " << lineNo << ": unknown section '" << words[0] << "'" << endl;
			errorCount++;
			section = SEC_SKIP;
			remaining = howMany2;
			if (file.eof())
				break;
			continue;
		}

		if (section == SEC_SKIP && remaining > 0) {
			remaining--;
			if (file.eof())
				break;
			continue;
		}

		if (section == SEC_NONE || remaining == 0) {
			cout << "  - line " << lineNo << ": unexpected line '" << words[0]
				<< "' (more records than the section count, or unknown keyword)" << endl;
			errorCount++;
			if (file.eof())
				break;
			continue;
		}

		remaining--;
		if (section == SEC_LOC) parseLocation(words, n, lineNo, sim);
		else if (section == SEC_CONN) parseConnection(words, n, lineNo, sim);
		else if (section == SEC_ROB) parseRobot(words, n, lineNo, sim);
		else if (section == SEC_REQ) parseRequest(words, n, lineNo, sim);
		else if (section == SEC_EVT) parseEvent(words, n, lineNo, sim);

		if (file.eof())
			break;
	}
	file.close();

	if (!sawEnd) {
		closeSection(section, remaining, declared, lineNo);
		cout << "  - file ended without an END line" << endl;
		errorCount++;
	}
	if (!haveMaxSteps) {
		cout << "  - PARAM max_steps is missing" << endl;
		errorCount++;
	}
	if (!seen[SEC_LOC]) { cout << "  - section LOCATIONS is missing" << endl; errorCount++; }
	if (!seen[SEC_CONN]) { cout << "  - section CONNECTIONS is missing" << endl; errorCount++; }
	if (!seen[SEC_ROB]) { cout << "  - section ROBOTS is missing" << endl; errorCount++; }

	// The cross-checks always run, so every problem in the file is reported in one go.
	validate(sim);

	if (errorCount > 0)
		return false;

	sim.env.buildAdjacency();
	sim.prepare();
	return true;
}
