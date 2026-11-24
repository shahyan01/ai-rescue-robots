#include "ScenarioLoader.h"
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

void ErrorList::add(const char* fmt, ...) {
	Message m;
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(m.text, sizeof(m.text), fmt, ap);
	va_end(ap);
	msgs.push(m);
}

void ErrorList::print(const char* title) const {
	printf("%s (%d):\n", title, msgs.size());
	for (int i = 0; i < msgs.size(); i++)
		printf("  - %s\n", msgs[i].text);
}

enum Section { SEC_NONE, SEC_LOC, SEC_CONN, SEC_ROB, SEC_REQ, SEC_EVT, SEC_SKIP };

static const int MAX_TOK = 16;
static const int TOK_LEN = 64;

static int tokenize(const char* line, char tok[][TOK_LEN]) {
	int n = 0;
	const char* p = line;
	while (*p && n < MAX_TOK) {
		while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
			p++;
		if (!*p)
			break;
		int k = 0;
		while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
			if (k < TOK_LEN - 1)
				tok[n][k++] = *p;
			p++;
		}
		tok[n][k] = '\0';
		n++;
	}
	return n;
}

static bool toInt(const char* s, int& v) {
	if (*s == '\0')
		return false;
	char* end;
	long x = strtol(s, &end, 10);
	if (*end != '\0' || x < -1000000000L || x > 1000000000L)
		return false;
	v = (int)x;
	return true;
}

static const char* sectionName(Section s) {
	switch (s) {
	case SEC_LOC: return "LOCATIONS";
	case SEC_CONN: return "CONNECTIONS";
	case SEC_ROB: return "ROBOTS";
	case SEC_REQ: return "REQUESTS";
	case SEC_EVT: return "EVENTS";
	default: return "";
	}
}

static bool isKeyword(const char* s) {
	return !strcmp(s, "SCENARIO") || !strcmp(s, "PARAM") || !strcmp(s, "LOCATIONS") ||
		!strcmp(s, "CONNECTIONS") || !strcmp(s, "ROBOTS") || !strcmp(s, "REQUESTS") ||
		!strcmp(s, "EVENTS") || !strcmp(s, "END");
}

static void parseLocation(char tok[][TOK_LEN], int n, int ln, Simulation& sim, ErrorList& errs) {
	Location l;
	if (n != 3) {
		errs.add("line %d: location needs 3 fields (id name type), found %d", ln, n);
		return;
	}
	if (!toInt(tok[0], l.id)) {
		errs.add("line %d: location id '%s' is not a number", ln, tok[0]);
		return;
	}
	if (strlen(tok[1]) >= sizeof(l.name)) {
		errs.add("line %d: location name '%s' is too long (max 31 characters)", ln, tok[1]);
		return;
	}
	strcpy(l.name, tok[1]);
	if (!parseLocType(tok[2], l.type)) {
		errs.add("line %d: unknown location type '%s'", ln, tok[2]);
		return;
	}
	l.stationUp = true;
	l.adjHead = -1;
	sim.env.locs.push(l);
}

static void parseConnection(char tok[][TOK_LEN], int n, int ln, Simulation& sim, ErrorList& errs) {
	Connection c;
	if (n != 5) {
		errs.add("line %d: connection needs 5 fields (id a b cost status), found %d", ln, n);
		return;
	}
	if (!toInt(tok[0], c.id) || !toInt(tok[1], c.aId) || !toInt(tok[2], c.bId) || !toInt(tok[3], c.cost)) {
		errs.add("line %d: connection id, endpoints and cost must be whole numbers", ln);
		return;
	}
	if (c.cost < 1) {
		errs.add("line %d: connection %d has cost %d, cost must be at least 1", ln, c.id, c.cost);
		return;
	}
	if (!strcmp(tok[4], "OPEN"))
		c.blocked = false;
	else if (!strcmp(tok[4], "BLOCKED"))
		c.blocked = true;
	else {
		errs.add("line %d: connection status '%s' must be OPEN or BLOCKED", ln, tok[4]);
		return;
	}
	c.aIdx = -1;
	c.bIdx = -1;
	sim.env.conns.push(c);
}

static void parseRobot(char tok[][TOK_LEN], int n, int ln, Simulation& sim, ErrorList& errs) {
	Robot r;
	if (n != 5) {
		errs.add("line %d: robot needs 5 fields (id start capability max initial), found %d", ln, n);
		return;
	}
	if (!toInt(tok[0], r.id) || !toInt(tok[1], r.location) || !toInt(tok[3], r.maxBattery) || !toInt(tok[4], r.battery)) {
		errs.add("line %d: robot id, start and battery values must be whole numbers", ln);
		return;
	}
	if (!parseCategory(tok[2], true, r.capability)) {
		errs.add("line %d: unknown robot capability '%s'", ln, tok[2]);
		return;
	}
	if (r.maxBattery < 1 || r.battery < 0 || r.battery > r.maxBattery) {
		errs.add("line %d: robot %d has battery %d with max %d (need 0 <= initial <= max, max >= 1)", ln, r.id, r.battery, r.maxBattery);
		return;
	}
	r.state = RS_AVAILABLE;
	r.online = true;
	sim.robots.push(r);
}

static void parseRequest(char tok[][TOK_LEN], int n, int ln, Simulation& sim, ErrorList& errs) {
	Request r;
	if (n != 6) {
		errs.add("line %d: request needs 6 fields (id time location category urgency duration), found %d", ln, n);
		return;
	}
	if (!toInt(tok[0], r.id) || !toInt(tok[1], r.reportTime) || !toInt(tok[2], r.location) ||
		!toInt(tok[4], r.urgency) || !toInt(tok[5], r.duration)) {
		errs.add("line %d: request numeric fields must be whole numbers", ln);
		return;
	}
	if (!parseCategory(tok[3], false, r.category)) {
		errs.add("line %d: unknown request category '%s'", ln, tok[3]);
		return;
	}
	if (r.reportTime < 0 || r.urgency < 1 || r.urgency > 5 || r.duration < 1) {
		errs.add("line %d: request %d needs time >= 0, urgency 1-5 and duration >= 1", ln, r.id);
		return;
	}
	r.state = RQ_NOT_REPORTED;
	r.assignedRobot = -1;
	sim.requests.push(r);
}

static void parseEvent(char tok[][TOK_LEN], int n, int ln, Simulation& sim, ErrorList& errs) {
	Event e;
	e.b = 0;
	if (n < 3) {
		errs.add("line %d: event needs at least 3 fields (time TYPE target)", ln);
		return;
	}
	if (!toInt(tok[0], e.time) || e.time < 0) {
		errs.add("line %d: event time '%s' must be a whole number >= 0", ln, tok[0]);
		return;
	}
	int expected = 3;
	if (!strcmp(tok[1], "BLOCK")) e.type = EV_BLOCK;
	else if (!strcmp(tok[1], "UNBLOCK")) e.type = EV_UNBLOCK;
	else if (!strcmp(tok[1], "STATION_DOWN")) e.type = EV_STATION_DOWN;
	else if (!strcmp(tok[1], "STATION_UP")) e.type = EV_STATION_UP;
	else if (!strcmp(tok[1], "ROBOT_OFFLINE")) e.type = EV_ROBOT_OFFLINE;
	else if (!strcmp(tok[1], "ROBOT_ONLINE")) e.type = EV_ROBOT_ONLINE;
	else if (!strcmp(tok[1], "SET_URGENCY")) { e.type = EV_SET_URGENCY; expected = 4; }
	else {
		errs.add("line %d: unknown event type '%s'", ln, tok[1]);
		return;
	}
	if (n != expected) {
		errs.add("line %d: %s needs %d fields, found %d", ln, tok[1], expected, n);
		return;
	}
	if (!toInt(tok[2], e.a) || (expected == 4 && !toInt(tok[3], e.b))) {
		errs.add("line %d: event target must be a whole number", ln);
		return;
	}
	if (e.type == EV_SET_URGENCY && (e.b < 1 || e.b > 5)) {
		errs.add("line %d: new urgency %d must be 1-5", ln, e.b);
		return;
	}
	sim.events.push(e);
}

static void parseParam(char tok[][TOK_LEN], int n, int ln, Simulation& sim, ErrorList& errs, bool& haveMaxSteps) {
	int v;
	if (n != 3 || !toInt(tok[2], v)) {
		errs.add("line %d: PARAM needs a key and a whole-number value", ln);
		return;
	}
	if (!strcmp(tok[1], "max_steps")) {
		if (v < 1) { errs.add("line %d: max_steps must be at least 1", ln); return; }
		sim.params.maxSteps = v;
		haveMaxSteps = true;
	} else if (!strcmp(tok[1], "speed")) {
		if (v < 1) { errs.add("line %d: speed must be at least 1", ln); return; }
		sim.params.speed = v;
	} else if (!strcmp(tok[1], "battery_per_cost_unit")) {
		if (v < 0) { errs.add("line %d: battery_per_cost_unit cannot be negative", ln); return; }
		sim.params.batteryPerCost = v;
	} else if (!strcmp(tok[1], "battery_per_work_step")) {
		if (v < 0) { errs.add("line %d: battery_per_work_step cannot be negative", ln); return; }
		sim.params.batteryPerWork = v;
	} else if (!strcmp(tok[1], "recharge_per_step")) {
		if (v < 0) { errs.add("line %d: recharge_per_step cannot be negative", ln); return; }
		sim.params.rechargePerStep = v;
	} else if (!strcmp(tok[1], "low_battery_percent")) {
		if (v < 0 || v > 100) { errs.add("line %d: low_battery_percent must be 0 to 100", ln); return; }
		sim.params.lowBatteryPercent = v;
	}
	// unknown keys are ignored so that extended files still load
}

static void closeSection(Section sec, int remaining, int declared, int ln, ErrorList& errs) {
	if (sec != SEC_NONE && sec != SEC_SKIP && remaining > 0)
		errs.add("line %d: %s declares %d records but only %d were found", ln, sectionName(sec), declared, declared - remaining);
}

// Stable merge sort by time. Records with equal time keep their file order.
static void sortEvents(DynArray<Event>& a) {
	int n = a.size();
	Event* tmp = new Event[n + 1];
	for (int width = 1; width < n; width *= 2) {
		for (int lo = 0; lo < n; lo += 2 * width) {
			int mid = (lo + width < n) ? lo + width : n;
			int hi = (lo + 2 * width < n) ? lo + 2 * width : n;
			int i = lo, j = mid, k = lo;
			while (i < mid && j < hi)
				tmp[k++] = (a[j].time < a[i].time) ? a[j++] : a[i++];
			while (i < mid) tmp[k++] = a[i++];
			while (j < hi) tmp[k++] = a[j++];
		}
		for (int i = 0; i < n; i++)
			a[i] = tmp[i];
	}
	delete[] tmp;
}

static void sortRequests(DynArray<Request>& a) {
	int n = a.size();
	Request* tmp = new Request[n + 1];
	for (int width = 1; width < n; width *= 2) {
		for (int lo = 0; lo < n; lo += 2 * width) {
			int mid = (lo + width < n) ? lo + width : n;
			int hi = (lo + 2 * width < n) ? lo + 2 * width : n;
			int i = lo, j = mid, k = lo;
			while (i < mid && j < hi)
				tmp[k++] = (a[j].reportTime < a[i].reportTime) ? a[j++] : a[i++];
			while (i < mid) tmp[k++] = a[i++];
			while (j < hi) tmp[k++] = a[j++];
		}
		for (int i = 0; i < n; i++)
			a[i] = tmp[i];
	}
	delete[] tmp;
}

static void validate(Simulation& sim, ErrorList& errs, ErrorList& warns) {
	Environment& env = sim.env;
	int bad = 0;

	bool locOk = env.indexLocations(bad);
	if (!locOk)
		errs.add("location id %d is duplicated or out of range (0 to %d)", bad, IdMap::MAX_ID);
	bool connOk = env.indexConnections(bad);
	if (!connOk)
		errs.add("connection id %d is duplicated or out of range (0 to %d)", bad, IdMap::MAX_ID);

	int n = sim.robots.size();
	int* ids = new int[n + 1];
	for (int i = 0; i < n; i++)
		ids[i] = sim.robots[i].id;
	if (!sim.robotMap.build(ids, n, bad))
		errs.add("robot id %d is duplicated or out of range", bad);
	delete[] ids;

	sortRequests(sim.requests);
	n = sim.requests.size();
	ids = new int[n + 1];
	for (int i = 0; i < n; i++)
		ids[i] = sim.requests[i].id;
	if (!sim.requestMap.build(ids, n, bad))
		errs.add("request id %d is duplicated or out of range", bad);
	delete[] ids;

	if (locOk) {
		for (int i = 0; i < env.conns.size(); i++) {
			const Connection& c = env.conns[i];
			if (env.locIndex(c.aId) == -1 || env.locIndex(c.bId) == -1)
				errs.add("connection %d refers to a location that does not exist (%d, %d)", c.id, c.aId, c.bId);
			else if (c.aId == c.bId)
				errs.add("connection %d connects location %d to itself", c.id, c.aId);
		}
		for (int i = 0; i < sim.robots.size(); i++)
			if (env.locIndex(sim.robots[i].location) == -1)
				errs.add("robot %d starts at location %d which does not exist", sim.robots[i].id, sim.robots[i].location);
		for (int i = 0; i < sim.requests.size(); i++)
			if (env.locIndex(sim.requests[i].location) == -1)
				errs.add("request %d is at location %d which does not exist", sim.requests[i].id, sim.requests[i].location);
	}

	sortEvents(sim.events);
	for (int i = 0; i < sim.events.size(); i++) {
		const Event& e = sim.events[i];
		switch (e.type) {
		case EV_BLOCK:
		case EV_UNBLOCK:
			if (connOk && env.connIndex(e.a) == -1)
				errs.add("event at time %d refers to unknown connection %d", e.time, e.a);
			break;
		case EV_STATION_DOWN:
		case EV_STATION_UP:
			if (locOk) {
				int li = env.locIndex(e.a);
				if (li == -1)
					errs.add("event at time %d refers to unknown location %d", e.time, e.a);
				else if (env.locs[li].type != LT_CHARGING)
					errs.add("event at time %d: location %d is not a charging station", e.time, e.a);
			}
			break;
		case EV_SET_URGENCY:
			if (sim.requestMap.find(e.a) == -1)
				errs.add("event at time %d refers to unknown request %d", e.time, e.a);
			break;
		case EV_ROBOT_OFFLINE:
		case EV_ROBOT_ONLINE:
			if (sim.robotMap.find(e.a) == -1)
				errs.add("event at time %d refers to unknown robot %d", e.time, e.a);
			break;
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
			warns.add("event %d at time %d is after max_steps %d and will never fire", i + 1, e.time, sim.params.maxSteps);
		if (e.type == EV_SET_URGENCY) {
			int q = sim.requestMap.find(e.a);
			if (q != -1) {
				if (urg[q] == e.b)
					warns.add("event at time %d sets request %d to urgency %d, which it already has", e.time, e.a, e.b);
				urg[q] = e.b;
			}
		}
	}
	for (int i = 0; i < nr; i++)
		if (sim.requests[i].reportTime >= sim.params.maxSteps)
			warns.add("request %d is reported at time %d, after max_steps %d, and will never be reported", sim.requests[i].id, sim.requests[i].reportTime, sim.params.maxSteps);
	delete[] urg;
}

bool loadScenario(const char* path, Simulation& sim, ErrorList& errs, ErrorList& warns) {
	FILE* f = fopen(path, "r");
	if (!f) {
		errs.add("cannot open file '%s'", path);
		return false;
	}

	char line[512];
	char tok[MAX_TOK][TOK_LEN];
	int ln = 0;
	Section sec = SEC_NONE;
	int remaining = 0;
	int declared = 0;
	bool seen[7] = {false, false, false, false, false, false, false};
	bool haveMaxSteps = false;
	bool sawEnd = false;

	while (fgets(line, sizeof(line), f)) {
		ln++;
		int len = (int)strlen(line);
		if (len == (int)sizeof(line) - 1 && line[len - 1] != '\n') {
			errs.add("line %d: line is too long", ln);
			int ch;
			while ((ch = fgetc(f)) != EOF && ch != '\n') {}
			continue;
		}
		const char* p = line;
		while (*p == ' ' || *p == '\t')
			p++;
		if (*p == '#' || *p == '\n' || *p == '\r' || *p == '\0')
			continue;

		if (sawEnd) {
			errs.add("line %d: text after END", ln);
			break;
		}

		int n = tokenize(line, tok);
		if (n == 0)
			continue;

		if (isKeyword(tok[0])) {
			closeSection(sec, remaining, declared, ln, errs);
			sec = SEC_NONE;
			remaining = 0;

			if (!strcmp(tok[0], "END")) {
				sawEnd = true;
				continue;
			} else if (!strcmp(tok[0], "SCENARIO")) {
				if (n >= 2) {
					strncpy(sim.scenarioName, tok[1], sizeof(sim.scenarioName) - 1);
					sim.scenarioName[sizeof(sim.scenarioName) - 1] = '\0';
				}
			} else if (!strcmp(tok[0], "PARAM")) {
				parseParam(tok, n, ln, sim, errs, haveMaxSteps);
			} else {
				Section s = !strcmp(tok[0], "LOCATIONS") ? SEC_LOC :
					!strcmp(tok[0], "CONNECTIONS") ? SEC_CONN :
					!strcmp(tok[0], "ROBOTS") ? SEC_ROB :
					!strcmp(tok[0], "REQUESTS") ? SEC_REQ : SEC_EVT;
				int cnt;
				if (n != 2 || !toInt(tok[1], cnt) || cnt < 0) {
					errs.add("line %d: %s header needs a count (e.g. %s 10)", ln, tok[0], tok[0]);
				} else {
					if (seen[s])
						errs.add("line %d: section %s appears twice", ln, tok[0]);
					seen[s] = true;
					sec = s;
					declared = cnt;
					remaining = cnt;
				}
			}
			continue;
		}

		int cnt2;
		bool upper = true;
		for (const char* c = tok[0]; *c; c++)
			if (*c < 'A' || *c > 'Z') upper = false;
		if ((sec == SEC_NONE || remaining == 0) && n == 2 && upper && toInt(tok[1], cnt2) && cnt2 >= 0) {
			errs.add("line %d: unknown section '%s'", ln, tok[0]);
			sec = SEC_SKIP;
			remaining = cnt2;
			continue;
		}

		if (sec == SEC_SKIP && remaining > 0) {
			remaining--;
			continue;
		}

		if (sec == SEC_NONE || remaining == 0) {
			errs.add("line %d: unexpected line '%s' (more records than the section count, or unknown keyword)", ln, tok[0]);
			continue;
		}

		remaining--;
		switch (sec) {
		case SEC_LOC: parseLocation(tok, n, ln, sim, errs); break;
		case SEC_CONN: parseConnection(tok, n, ln, sim, errs); break;
		case SEC_ROB: parseRobot(tok, n, ln, sim, errs); break;
		case SEC_REQ: parseRequest(tok, n, ln, sim, errs); break;
		case SEC_EVT: parseEvent(tok, n, ln, sim, errs); break;
		default: break;
		}
	}
	fclose(f);

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
