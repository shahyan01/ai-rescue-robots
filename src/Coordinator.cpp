#include "Simulation.h"

bool Simulation::isIdle(int i)
{
	Robot &b = robots[i];

	if (!b.online || b.request != -1 || b.roadTo != -1)
	{
		return false;
	}

	return b.state == RS_AVAILABLE || b.state == RS_RECHARGING;
}

// Assignment round. Pending requests are taken in priority order (urgency, report
// time, id). For each one, every idle robot that can serve the category and has enough
// battery is a candidate. The winner is a specialist before a GENERAL robot, then the
// lowest route cost, then the lowest robot id. A request nobody can take is skipped
// and does not block the ones behind it.

void Simulation::assignRound()
{
	if (pending.getSize() == 0)
	{
		return;
	}

	bool *idle = new bool[numRobots + 1];
	bool anyIdle = false;

	for (int i = 0 ; i < numRobots ; i++)
	{
		idle[i] = isIdle(i);
		if (idle[i])
		{
			anyIdle = true;
		}
	}

	if (!anyIdle)
	{
		delete[] idle;
		return;
	}

	int howMany = pending.getSize();
	int *order = new int[howMany + 1];

	for (int k = 0 ; k < howMany ; k++)
	{
		order[k] = pending.pop();
	}

	int *wanted = new int[numRobots + 1];
	bool *taken = new bool[numRobots + 1];

	for (int i = 0 ; i < numRobots ; i++)
	{
		wanted[i] = 0;
		taken[i] = false;
	}

	Tree *charger = env.chargerTree();

	for (int k = 0 ; k < howMany ; k++)
	{
		int q = order[k];
		Request &r = requests[q];
		Tree* tree = 0;

		int best = -1;
		int bestCost = 0;
		int eligible = 0;

		int reserve = charger->dist[r.place];

		if (reserve >= NO_ROUTE)
		{
			reserve = 0;
		}

		for (int i = 0 ; i < numRobots ; i++)
		{
			if (!idle[i])
			{
				continue;
			}

			Robot& b = robots[i];

			if (b.capability != CAT_GENERAL && b.capability != r.category)
			{
				continue;
			}

			if (tree == 0)
			{
				tree = env.treeFor(r.place);
			}

			int d = tree->dist[b.at];

			if (d >= NO_ROUTE)
			{
				continue;
			}

			int need = (d + reserve) * params.batteryPerCost + r.duration * params.batteryPerWork;

			if (b.battery < need)
			{
				continue;
			}

			wanted[i]++;

			if (taken[i])
			{
				continue;
			}

			eligible++;

			bool better = false;

			if (best == -1)
			{
				better = true;
			}
			
			else
			{
				bool thisIsSpecialist = (b.capability != CAT_GENERAL);
				bool bestIsSpecialist = (robots[best].capability != CAT_GENERAL);

				if (thisIsSpecialist != bestIsSpecialist)
				{
					better = thisIsSpecialist;
				}

				else if (d != bestCost)
				{
					better = (d < bestCost);
				}

				else
				{
					better = (b.id < robots[best].id);
				}
			}

			if (better)
			{
				best = i;
				bestCost = d;
			}
		}

		if (best == -1)
		{
			continue;
		}

		if (eligible >= 2)
		{
			stats.conflicts++;
			logStart("CONFLICT", "request", r.id, "-", "-");
			logFile << eligible << " eligible robots, robot " << robots[best].id << " chosen" << endl;
		}

		Robot &winner = robots[best];
		taken[best] = true;
		winner.request = q;
		winner.expectRemain = bestCost;
		r.assignedRobot = best;
		setRequestState(q, RQ_ASSIGNED, "robot assigned");
		setRobotState(best, RS_TRAVELLING, "going to request");
		stats.assignments++;
		logStart("ASSIGN", "request", r.id, "-", "robot");
		logFile << "robot = " << winner.id << " route_cost = " << bestCost << " battery = " << winner.battery << endl;
		order[k] = -1;
	}

	// A robot that two or more requests wanted in this round is a conflict too.

	for (int i = 0 ; i < numRobots ; i++)
	{
		if (wanted[i] >= 2)
		{
			stats.conflicts++;
			logStart("CONFLICT", "robot", robots[i].id, "-", "-");
			logFile << "wanted by " << wanted[i] << " requests" << endl;
		}
	}

	// Requests that got no robot go back into the heap.

	for (int k = 0 ; k < howMany ; k++)
	{
		if (order[k] != -1)
		{
			pending.push(order[k]);
		}
	}

	delete[] idle;
	delete[] order;
	delete[] wanted;
	delete[] taken;
}