#include "Simulation.h"

void Simulation::printEnvironment() const {
	int typeCount[6] = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < env.locs.size(); i++)
		typeCount[env.locs[i].type]++;
	printf("Time step: %d / %d\n", now, params.maxSteps);
	printf("Locations: %d  Connections: %d  Blocked: %d\n", env.locs.size(), env.conns.size(), env.countBlocked());
	printf("Types: JUNCTION %d, BUILDING %d, HOSPITAL %d, DEPOT %d, CHARGING %d, COMMAND %d\n",
		typeCount[LT_JUNCTION], typeCount[LT_BUILDING], typeCount[LT_HOSPITAL],
		typeCount[LT_DEPOT], typeCount[LT_CHARGING], typeCount[LT_COMMAND]);

	printf("Blocked connections:");
	int shown = 0;
	for (int i = 0; i < env.conns.size(); i++)
		if (env.conns[i].blocked) {
			printf(" %d(%d-%d)", env.conns[i].id, env.conns[i].aId, env.conns[i].bId);
			shown++;
		}
	if (shown == 0)
		printf(" none");
	printf("\n");

	printf("Charging stations down:");
	shown = 0;
	for (int i = 0; i < env.locs.size(); i++)
		if (env.locs[i].type == LT_CHARGING && !env.locs[i].stationUp) {
			printf(" %d", env.locs[i].id);
			shown++;
		}
	if (shown == 0)
		printf(" none");
	printf("\n");

	printf("Cut-off locations (no open connection):");
	shown = 0;
	for (int i = 0; i < env.locs.size(); i++)
		if (env.countOpenEdges(i) == 0) {
			printf(" %d(%s)", env.locs[i].id, env.locs[i].name);
			shown++;
		}
	if (shown == 0)
		printf(" none");
	printf("\n");
}

void Simulation::printRobots() const {
	printf("%-4s %-5s %-9s %-11s %-12s %-7s %s\n", "ID", "Loc", "Capab.", "Battery", "State", "Online", "Request");
	for (int i = 0; i < robots.size(); i++) {
		const Robot& b = robots[i];
		printf("%-4d %-5d %-9s %3d/%-7d %-12s %-7s ", b.id, env.locs[b.at].id, categoryName(b.capability),
			b.battery, b.maxBattery, robotStateName(b.state), b.online ? "yes" : "no");
		if (b.request >= 0)
			printf("%d\n", requests[b.request].id);
		else
			printf("-\n");
	}
}

void Simulation::printRequests() const {
	int counts[6] = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < requests.size(); i++)
		counts[requests[i].state]++;
	printf("Not reported: %d  Pending: %d  Assigned: %d  In progress: %d  Completed: %d  Failed: %d\n",
		counts[RQ_NOT_REPORTED], counts[RQ_PENDING], counts[RQ_ASSIGNED],
		counts[RQ_IN_PROGRESS], counts[RQ_COMPLETED], counts[RQ_FAILED]);
	printf("%-4s %-6s %-5s %-9s %-4s %-4s %-12s %s\n", "ID", "Time", "Loc", "Category", "Urg", "Dur", "State", "Robot");
	for (int i = 0; i < requests.size(); i++) {
		const Request& r = requests[i];
		if (r.state == RQ_NOT_REPORTED)
			continue;
		printf("%-4d %-6d %-5d %-9s %-4d %-4d %-12s ", r.id, r.reportTime, r.location,
			categoryName(r.category), r.urgency, r.duration, reqStateName(r.state));
		if (r.assignedRobot >= 0)
			printf("%d\n", robots[r.assignedRobot].id);
		else
			printf("-\n");
	}
}

void Simulation::printRoute(int fromId, int toId) {
	int n = env.locs.size();
	int* path = new int[n + 1];
	int len = 0;
	int cost = 0;
	if (env.locIndex(fromId) == -1 || env.locIndex(toId) == -1) {
		printf("Unknown location id.\n");
	} else if (env.shortestPath(fromId, toId, path, len, cost)) {
		printf("Route cost %d, %d stops:", cost, len);
		for (int i = 0; i < len; i++)
			printf(" %d", env.locs[path[i]].id);
		printf("\n");
	} else {
		printf("No route from %d to %d with the current blocked connections.\n", fromId, toId);
	}
	delete[] path;
}

// Final statistics. Execution time is printed here only, never in the log,
// so two runs of the same file give identical logs.
void Simulation::printStats(FILE* out) const {
	int counts[6] = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < requests.size(); i++)
		counts[requests[i].state]++;
	int remaining = requests.size() - counts[RQ_COMPLETED] - counts[RQ_FAILED];

	int served = 0;
	long respSum = 0;
	int respMax = 0;
	for (int i = 0; i < requests.size(); i++) {
		const Request& r = requests[i];
		if (r.state != RQ_COMPLETED || r.arriveTime < 0)
			continue;
		int resp = r.arriveTime - r.reportTime;
		respSum += resp;
		if (resp > respMax)
			respMax = resp;
		served++;
	}

	long moved = 0, consumed = 0, recharged = 0;
	double utilSum = 0;
	for (int i = 0; i < robots.size(); i++) {
		const Robot& b = robots[i];
		moved += b.moved;
		consumed += b.consumed;
		recharged += b.recharged;
		if (b.onlineSteps > 0)
			utilSum += (double)(b.travelSteps + b.workSteps) / b.onlineSteps;
	}

	double secs = (double)(clock() - startClock) / CLOCKS_PER_SEC;

	fprintf(out, "Scenario: %s\n", scenarioName);
	fprintf(out, "Run ended at step %d (max_steps %d)\n", endStep, params.maxSteps);
	fprintf(out, "Requests: %d total, %d completed, %d failed, %d remaining\n",
		requests.size(), counts[RQ_COMPLETED], counts[RQ_FAILED], remaining);
	if (served > 0)
		fprintf(out, "Response time: average %.2f, maximum %d (over %d served requests)\n", (double)respSum / served, respMax, served);
	else
		fprintf(out, "Response time: no request was served\n");
	if (robots.size() > 0)
		fprintf(out, "Utilisation: fleet mean %.1f%%\n", 100.0 * utilSum / robots.size());
	fprintf(out, "Movement cost: %ld cost units travelled by all robots\n", moved);
	fprintf(out, "Battery: %ld consumed, %ld recharged\n", consumed, recharged);
	fprintf(out, "Conflicts: %d, reassignments: %d, assignments: %d\n", stats.conflicts, stats.reassignments, stats.assignments);
	fprintf(out, "Events applied: %d, rejected: %d, invalid transitions rejected: %d\n",
		stats.eventsApplied, stats.eventsRejected, stats.badTransitions);
	fprintf(out, "Shortest-path searches: %d\n", env.treeBuilds);
	fprintf(out, "Execution time: %.4f s\n", secs);

	fprintf(out, "Per robot (id, capability, utilisation, travel steps, work steps, moved, consumed, recharged, final battery):\n");
	for (int i = 0; i < robots.size(); i++) {
		const Robot& b = robots[i];
		double u = (b.onlineSteps > 0) ? 100.0 * (b.travelSteps + b.workSteps) / b.onlineSteps : 0.0;
		fprintf(out, "  robot %d %s %.1f%% %d %d %d %d %d %d/%d\n", b.id, categoryName(b.capability), u,
			b.travelSteps, b.workSteps, b.moved, b.consumed, b.recharged, b.battery, b.maxBattery);
	}
}
