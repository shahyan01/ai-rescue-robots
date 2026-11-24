#include "Simulation.h"
#include <iomanip>

void Simulation::printEnvironment() const {
    int typeCount[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < env.locs.size(); i++)
        typeCount[env.locs[i].type]++;

    cout << "Time step: " << now << " / " << params.maxSteps << endl;
    cout << "Locations: " << env.locs.size() << "  Connections: " << env.conns.size()
         << "  Blocked: " << env.countBlocked() << endl;
    cout << "Types: JUNCTION " << typeCount[LT_JUNCTION]
         << ", BUILDING " << typeCount[LT_BUILDING]
         << ", HOSPITAL " << typeCount[LT_HOSPITAL]
         << ", DEPOT " << typeCount[LT_DEPOT]
         << ", CHARGING " << typeCount[LT_CHARGING]
         << ", COMMAND " << typeCount[LT_COMMAND] << endl;

    cout << "Blocked connections:";
    int shown = 0;
    for (int i = 0; i < env.conns.size(); i++)
        if (env.conns[i].blocked) {
            cout << " " << env.conns[i].id << "(" << env.conns[i].aId << "-" << env.conns[i].bId << ")";
            shown++;
        }
    if (shown == 0)
        cout << " none";
    cout << endl;

    cout << "Charging stations down:";
    shown = 0;
    for (int i = 0; i < env.locs.size(); i++)
        if (env.locs[i].type == LT_CHARGING && !env.locs[i].stationUp) {
            cout << " " << env.locs[i].id;
            shown++;
        }
    if (shown == 0)
        cout << " none";
    cout << endl;

    cout << "Cut-off locations (no open connection):";
    shown = 0;
    for (int i = 0; i < env.locs.size(); i++)
        if (env.countOpenEdges(i) == 0) {
            cout << " " << env.locs[i].id << "(" << env.locs[i].name << ")";
            shown++;
        }
    if (shown == 0)
        cout << " none";
    cout << endl;
}

void Simulation::printRobots() const {
    cout << left;
    cout << setw(5) << "ID" << setw(6) << "Loc" << setw(10) << "Capab."
         << setw(12) << "Battery" << setw(13) << "State" << setw(8) << "Online" << "Request" << endl;

    for (int i = 0; i < robots.size(); i++) {
        const Robot& b = robots[i];
        string battery = to_string(b.battery) + "/" + to_string(b.maxBattery);
        string online = "no";
        if (b.online)
            online = "yes";

        cout << setw(5) << b.id << setw(6) << env.locs[b.at].id << setw(10) << categoryName(b.capability)
             << setw(12) << battery << setw(13) << robotStateName(b.state) << setw(8) << online;
        if (b.request >= 0)
            cout << requests[b.request].id << endl;
        else
            cout << "-" << endl;
    }
}

void Simulation::printRequests() const {
    int counts[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < requests.size(); i++)
        counts[requests[i].state]++;

    cout << "Not reported: " << counts[RQ_NOT_REPORTED]
         << "  Pending: " << counts[RQ_PENDING]
         << "  Assigned: " << counts[RQ_ASSIGNED]
         << "  In progress: " << counts[RQ_IN_PROGRESS]
         << "  Completed: " << counts[RQ_COMPLETED]
         << "  Failed: " << counts[RQ_FAILED] << endl;

    cout << left;
    cout << setw(5) << "ID" << setw(7) << "Time" << setw(6) << "Loc" << setw(10) << "Category"
         << setw(5) << "Urg" << setw(5) << "Dur" << setw(13) << "State" << "Robot" << endl;

    for (int i = 0; i < requests.size(); i++) {
        const Request& r = requests[i];
        if (r.state == RQ_NOT_REPORTED)
            continue;
        cout << setw(5) << r.id << setw(7) << r.reportTime << setw(6) << r.location
             << setw(10) << categoryName(r.category) << setw(5) << r.urgency << setw(5) << r.duration
             << setw(13) << reqStateName(r.state);
        if (r.assignedRobot >= 0)
            cout << robots[r.assignedRobot].id << endl;
        else
            cout << "-" << endl;
    }
}

void Simulation::printRoute(int fromId, int toId) {
    int n = env.locs.size();
    int* path = new int[n + 1];
    int len = 0;
    int cost = 0;

    if (env.locIndex(fromId) == -1 || env.locIndex(toId) == -1) {
        cout << "Unknown location id." << endl;
    } else if (env.shortestPath(fromId, toId, path, len, cost)) {
        cout << "Route cost " << cost << ", " << len << " stops:";
        for (int i = 0; i < len; i++)
            cout << " " << env.locs[path[i]].id;
        cout << endl;
    } else {
        cout << "No route from " << fromId << " to " << toId << " with the current blocked connections." << endl;
    }
    delete[] path;
}

// Final statistics. Execution time is printed here only, never in the log,
// so two runs of the same file give identical logs.
void Simulation::printStats(ostream& out) const {
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

    long moved = 0;
    long consumed = 0;
    long recharged = 0;
    double utilSum = 0;
    for (int i = 0; i < robots.size(); i++) {
        const Robot& b = robots[i];
        moved += b.moved;
        consumed += b.consumed;
        recharged += b.recharged;
        if (b.onlineSteps > 0)
            utilSum += (double)(b.travelSteps + b.workSteps) / b.onlineSteps;
    }

    chrono::duration<double> elapsed = chrono::steady_clock::now() - startClock;
    double secs = elapsed.count();

    out << fixed;
    out << "Scenario: " << scenarioName << "\n";
    out << "Run ended at step " << endStep << " (max_steps " << params.maxSteps << ")\n";
    out << "Requests: " << requests.size() << " total, " << counts[RQ_COMPLETED] << " completed, "
        << counts[RQ_FAILED] << " failed, " << remaining << " remaining\n";

    if (served > 0)
        out << "Response time: average " << setprecision(2) << (double)respSum / served
            << ", maximum " << respMax << " (over " << served << " served requests)\n";
    else
        out << "Response time: no request was served\n";

    if (robots.size() > 0)
        out << "Utilisation: fleet mean " << setprecision(1) << 100.0 * utilSum / robots.size() << "%\n";

    out << "Movement cost: " << moved << " cost units travelled by all robots\n";
    out << "Battery: " << consumed << " consumed, " << recharged << " recharged\n";
    out << "Conflicts: " << stats.conflicts << ", reassignments: " << stats.reassignments
        << ", assignments: " << stats.assignments << "\n";
    out << "Events applied: " << stats.eventsApplied << ", rejected: " << stats.eventsRejected
        << ", invalid transitions rejected: " << stats.badTransitions << "\n";
    out << "Shortest-path searches: " << env.treeBuilds << "\n";
    out << "Execution time: " << setprecision(4) << secs << " s\n";

    out << "Per robot (id, capability, utilisation, travel steps, work steps, moved, consumed, recharged, final battery):\n";
    for (int i = 0; i < robots.size(); i++) {
        const Robot& b = robots[i];
        double u = 0.0;
        if (b.onlineSteps > 0)
            u = 100.0 * (b.travelSteps + b.workSteps) / b.onlineSteps;
        out << "  robot " << b.id << " " << categoryName(b.capability) << " "
            << setprecision(1) << u << "% "
            << b.travelSteps << " " << b.workSteps << " " << b.moved << " "
            << b.consumed << " " << b.recharged << " " << b.battery << "/" << b.maxBattery << "\n";
    }
}
