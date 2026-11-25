#include "Simulation.h"
#include "Utils.h"
#include <iomanip>

void Simulation::printEnvironment() {
	int typeCount[6] = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < env.numLocs; i++)
		typeCount[env.locs[i].type]++;
	cout << "Time step: " << now << " / " << params.maxSteps << endl;
	cout << "Locations: " << env.numLocs << "  Connections: " << env.numConns
		<< "  Blocked: " << env.countBlocked() << endl;
	cout << "Types: JUNCTION " << typeCount[LT_JUNCTION] << ", BUILDING " << typeCount[LT_BUILDING]
		<< ", HOSPITAL " << typeCount[LT_HOSPITAL] << ", DEPOT " << typeCount[LT_DEPOT]
		<< ", CHARGING " << typeCount[LT_CHARGING] << ", COMMAND " << typeCount[LT_COMMAND] << endl;

	cout << "Blocked connections:";
	int shown = 0;
	for (int i = 0; i < env.numConns; i++)
		if (env.conns[i].blocked) {
			cout << " " << env.conns[i].id << "(" << env.conns[i].aId << "-" << env.conns[i].bId << ")";
			shown++;
		}
	if (shown == 0)
		cout << " none";
	cout << endl;

	cout << "Charging stations down:";
	shown = 0;
	for (int i = 0; i < env.numLocs; i++)
		if (env.locs[i].type == LT_CHARGING && !env.locs[i].stationUp) {
			cout << " " << env.locs[i].id;
			shown++;
		}
	if (shown == 0)
		cout << " none";
	cout << endl;

	cout << "Cut-off locations (no open connection):";
	shown = 0;
	for (int i = 0; i < env.numLocs; i++)
		if (env.countOpenRoads(i) == 0) {
			cout << " " << env.locs[i].id << "(" << env.locs[i].name << ")";
			shown++;
		}
	if (shown == 0)
		cout << " none";
	cout << endl;
}

void Simulation::printRobots() {
	cout << left << setw(5) << "ID" << setw(6) << "Loc" << setw(10) << "Capab."
		<< setw(10) << "Battery" << setw(13) << "State" << setw(8) << "Online" << "Request" << endl;
	for (int i = 0; i < numRobots; i++) {
		Robot& b = robots[i];
		cout << left << setw(5) << b.id << setw(6) << env.locs[b.at].id << setw(10) << categoryName(b.capability);
		cout << right << setw(3) << b.battery << "/" << left << setw(6) << b.maxBattery;
		cout << setw(13) << robotStateName(b.state) << setw(8) << (b.online ? "yes" : "no");
		if (b.request >= 0)
			cout << requests[b.request].id << endl;
		else
			cout << "-" << endl;
	}
}

void Simulation::printRequests() {
	int counts[6] = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < numRequests; i++)
		counts[requests[i].state]++;
	cout << "Not reported: " << counts[RQ_NOT_REPORTED] << "  Pending: " << counts[RQ_PENDING]
		<< "  Assigned: " << counts[RQ_ASSIGNED] << "  In progress: " << counts[RQ_IN_PROGRESS]
		<< "  Completed: " << counts[RQ_COMPLETED] << "  Failed: " << counts[RQ_FAILED] << endl;
	cout << left << setw(5) << "ID" << setw(7) << "Time" << setw(6) << "Loc" << setw(10) << "Category"
		<< setw(5) << "Urg" << setw(5) << "Dur" << setw(13) << "State" << "Robot" << endl;
	for (int i = 0; i < numRequests; i++) {
		Request& r = requests[i];
		if (r.state == RQ_NOT_REPORTED)
			continue;
		cout << left << setw(5) << r.id << setw(7) << r.reportTime << setw(6) << r.location
			<< setw(10) << categoryName(r.category) << setw(5) << r.urgency << setw(5) << r.duration
			<< setw(13) << requestStateName(r.state);
		if (r.assignedRobot >= 0)
			cout << robots[r.assignedRobot].id << endl;
		else
			cout << "-" << endl;
	}
}

void Simulation::printRoute(int fromId, int toId) {
	if (env.findLocation(fromId) == -1 || env.findLocation(toId) == -1) {
		cout << "Unknown location id." << endl;
		return;
	}
	int* path = new int[env.numLocs + 1];
	int length = 0;
	int cost = 0;
	if (env.shortestPath(fromId, toId, path, length, cost)) {
		cout << "Route cost " << cost << ", " << length << " stops:";
		for (int i = 0; i < length; i++)
			cout << " " << env.locs[path[i]].id;
		cout << endl;
	} else {
		cout << "No route from " << fromId << " to " << toId << " with the current blocked connections." << endl;
	}
	delete[] path;
}

// Final statistics. The execution time is printed here only, never in the log,
// so two runs of the same input give identical logs.
void Simulation::printStats(ostream& out) {
	int counts[6] = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < numRequests; i++)
		counts[requests[i].state]++;
	int remaining = numRequests - counts[RQ_COMPLETED] - counts[RQ_FAILED];

	int served = 0;
	int responseSum = 0;
	int responseMax = 0;
	for (int i = 0; i < numRequests; i++) {
		Request& r = requests[i];
		if (r.state != RQ_COMPLETED || r.arriveTime < 0)
			continue;
		int response = r.arriveTime - r.reportTime;
		responseSum += response;
		if (response > responseMax)
			responseMax = response;
		served++;
	}

	int moved = 0;
	int consumed = 0;
	int recharged = 0;
	double utilisationSum = 0;
	for (int i = 0; i < numRobots; i++) {
		Robot& b = robots[i];
		moved += b.moved;
		consumed += b.consumed;
		recharged += b.recharged;
		if (b.onlineSteps > 0)
			utilisationSum += (double)(b.travelSteps + b.workSteps) / b.onlineSteps;
	}

	double seconds = (double)(clock() - startClock) / CLOCKS_PER_SEC;

	out << fixed;
	out << "Scenario: " << scenarioName << endl;
	out << "Run ended at step " << endStep << " (max_steps " << params.maxSteps << ")" << endl;
	out << "Requests: " << numRequests << " total, " << counts[RQ_COMPLETED] << " completed, "
		<< counts[RQ_FAILED] << " failed, " << remaining << " remaining" << endl;
	if (served > 0)
		out << "Response time: average " << setprecision(2) << (double)responseSum / served
			<< ", maximum " << responseMax << " (over " << served << " served requests)" << endl;
	else
		out << "Response time: no request was served" << endl;
	if (numRobots > 0)
		out << "Utilisation: fleet mean " << setprecision(1) << 100.0 * utilisationSum / numRobots << "%" << endl;
	out << "Movement cost: " << moved << " cost units travelled by all robots" << endl;
	out << "Battery: " << consumed << " consumed, " << recharged << " recharged" << endl;
	out << "Conflicts: " << stats.conflicts << ", reassignments: " << stats.reassignments
		<< ", assignments: " << stats.assignments << endl;
	out << "Events applied: " << stats.eventsApplied << ", rejected: " << stats.eventsRejected
		<< ", invalid transitions rejected: " << stats.badTransitions << endl;
	out << "Shortest-path searches: " << env.treeBuilds << endl;
	out << "Execution time: " << setprecision(4) << seconds << " s" << endl;

	out << "Per robot (id, capability, utilisation, travel steps, work steps, moved, consumed, recharged, final battery):" << endl;
	for (int i = 0; i < numRobots; i++) {
		Robot& b = robots[i];
		double u = 0.0;
		if (b.onlineSteps > 0)
			u = 100.0 * (b.travelSteps + b.workSteps) / b.onlineSteps;
		out << "  robot " << b.id << " " << categoryName(b.capability) << " " << setprecision(1) << u << "% "
			<< b.travelSteps << " " << b.workSteps << " " << b.moved << " " << b.consumed << " "
			<< b.recharged << " " << b.battery << "/" << b.maxBattery << endl;
	}
}
