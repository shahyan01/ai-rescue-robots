#include "Simulation.h"

// A robot can take a new request only when it is online, standing on a location
// (not on a road), holds no request, and is AVAILABLE or RECHARGING.
bool Simulation::isIdle(int i) const {
    const Robot& b = robots[i];
    if (!b.online || b.request != -1 || b.roadTo != -1)
        return false;
    return b.state == RS_AVAILABLE || b.state == RS_RECHARGING;
}

// Assignment round. Pending requests are taken in priority order (urgency, report
// time, id). For each one, every idle robot that can serve the category and has enough
// battery is a candidate. The winner is a specialist before a GENERAL robot, then the
// lowest route cost, then the lowest robot id. A request nobody can take is skipped and
// does not block the ones behind it.
void Simulation::assignRound() {
    if (pending.size() == 0)
        return;

    int R = robots.size();
    bool* idle = new bool[R + 1];
    bool any = false;
    for (int i = 0; i < R; i++) {
        idle[i] = isIdle(i);
        if (idle[i])
            any = true;
    }
    if (!any) {
        delete[] idle;
        return;
    }

    int P = pending.size();
    int* order = new int[P + 1];
    for (int k = 0; k < P; k++)
        order[k] = pending.pop();

    int* wanted = new int[R + 1];
    bool* taken = new bool[R + 1];
    for (int i = 0; i < R; i++) {
        wanted[i] = 0;
        taken[i] = false;
    }

    const Tree& charger = env.chargerTree();

    for (int k = 0; k < P; k++) {
        int q = order[k];
        Request& r = requests[q];
        const Tree* tree = 0;

        int best = -1;
        int bestCost = 0;
        int eligible = 0;

        // The battery reserve after the job: distance from the request to the
        // nearest working charger. Zero when none can be reached.
        int reserve = charger.dist[r.place];
        if (reserve >= INF_COST)
            reserve = 0;

        for (int i = 0; i < R; i++) {
            if (!idle[i])
                continue;
            const Robot& b = robots[i];
            if (b.capability != CAT_GENERAL && b.capability != r.category)
                continue;
            if (!tree)
                tree = &env.treeFor(r.place);
            int d = tree->dist[b.at];
            if (d >= INF_COST)
                continue;
            int need = (d + reserve) * params.batteryPerCost + r.duration * params.batteryPerWork;
            if (b.battery < need)
                continue;

            wanted[i]++;
            if (taken[i])
                continue;
            eligible++;

            bool better = false;
            if (best == -1) {
                better = true;
            } else {
                bool iSpecial = (b.capability != CAT_GENERAL);
                bool bestSpecial = (robots[best].capability != CAT_GENERAL);
                if (iSpecial != bestSpecial)
                    better = iSpecial;
                else if (d != bestCost)
                    better = (d < bestCost);
                else
                    better = (b.id < robots[best].id);
            }
            if (better) {
                best = i;
                bestCost = d;
            }
        }

        if (best == -1)
            continue;

        if (eligible >= 2) {
            stats.conflicts++;
            logLine("CONFLICT", "request", r.id, "-", "-",
                to_string(eligible) + " eligible robots, robot " + to_string(robots[best].id) + " chosen");
        }

        Robot& b = robots[best];
        taken[best] = true;
        b.request = q;
        b.expectRemain = bestCost;
        r.assignedRobot = best;
        setRequestState(q, RQ_ASSIGNED, "robot assigned");
        setRobotState(best, RS_TRAVELLING, "going to request");
        stats.assignments++;
        logLine("ASSIGN", "request", r.id, "-", "robot",
            "robot=" + to_string(b.id) + " route_cost=" + to_string(bestCost) + " battery=" + to_string(b.battery));
        order[k] = -1;
    }

    // A robot that several requests wanted in this round is a conflict too.
    for (int i = 0; i < R; i++) {
        if (wanted[i] >= 2) {
            stats.conflicts++;
            logLine("CONFLICT", "robot", robots[i].id, "-", "-",
                "wanted by " + to_string(wanted[i]) + " requests");
        }
    }

    for (int k = 0; k < P; k++)
        if (order[k] != -1)
            pending.push(order[k]);

    delete[] idle;
    delete[] order;
    delete[] wanted;
    delete[] taken;
}
