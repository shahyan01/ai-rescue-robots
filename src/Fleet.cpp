#include "Simulation.h"

void Simulation::spend(Robot &b, int amount)
{
	int used = amount;

	if (used > b.battery)
	{
		used = b.battery;
	}
	
	b.battery -= used;
	b.consumed += used;
}

bool Simulation::atWorkingStation(Robot &b)
{
	Location &l = env.locs[b.at];
	return l.type == LT_CHARGING && l.stationUp;
}

void Simulation::startRoad(Robot &b, int conn, int to)
{
	Connection &c = env.conns[conn];
	logStart("MOVE", "robot", b.id, "-", "-");
	logFile << "from = " << env.locs[b.at].id << " to = " << env.locs[to].id << " conn = " << c.id << " cost = " << c.cost << endl;
	b.roadTo = to;
	b.roadRemain = c.cost;
}

// Returns true when the robot reaches the end of the road.

bool Simulation::moveAlongRoad(Robot &b)
{
	int m = params.speed;

	if (m > b.roadRemain)
	{
		m = b.roadRemain;
	}

	b.roadRemain -= m;
	b.moved += m;

	spend(b, m * params.batteryPerCost);

	if (b.roadRemain > 0)
	{
		return false;
	}

	b.at = b.roadTo;
	b.roadTo = -1;

	return true;
}

void Simulation::advanceRobots()
{
	for (int i = 0 ; i < numRobots ; i++)
	{
		Robot &b = robots[i];

		if (!b.online)
		{
			continue;
		}

		// A robot that came back online in the middle of a road finishes the road first.

		if (b.roadTo != -1 && b.request == -1 && b.state == RS_AVAILABLE)
		{
			moveAlongRoad(b);
			continue;
		}

		if (b.state == RS_TRAVELLING)
		{
			travel(i);
		}

		else if (b.state == RS_PERFORMING)
		{
			doWork(i);
		}

		else if (b.state == RS_RETURNING)
		{
			returnTrip(i);
		}

		else if (b.state == RS_RECHARGING)
		{
			if (atWorkingStation(b))
			{
				rechargeStep(i);
			}

			else
			{
				setRobotState(i, RS_AVAILABLE, "station not working");
			}
		}
		
		else if (b.state == RS_AVAILABLE)
		{
			idleStep(i);
		}
	}
}

// At every location the next road comes from a fresh tree,\
// so a road that closed causes a detour and a road that reopened is used.

void Simulation::travel(int i)
{
	Robot &b = robots[i];
	Request &q = requests[b.request];

	if (b.roadTo == -1)
	{
		if (b.at == q.place)
		{
			arrive(i);
			return;
		}

		Tree *t = env.treeFor(q.place);
		int d = t->dist[b.at];

		if (d >= NO_ROUTE || b.battery < d * params.batteryPerCost)
		{
			logStart("WARN", "robot", b.id, "-", "-");
			logFile << "no usable route to request " << q.id << " from location " << env.locs[b.at].id << ", request goes back to pending" << endl;
			releaseRequestOf(i, "route lost");
			setRobotState(i, RS_AVAILABLE, "route lost");
			return;
		}

		if (b.expectRemain >= 0 && d > b.expectRemain)
		{
			logStart("REROUTE", "robot", b.id, "-", "-");
			logFile << "request " << q.id << " : route left grew from " << b.expectRemain << " to " << d << " at location " << env.locs[b.at].id << endl;
		}

		int nextStop = t->parent[b.at];
		startRoad(b, t->pconn[b.at], nextStop);
		b.expectRemain = t->dist[nextStop];
	}

	b.travelSteps++;

	if (moveAlongRoad(b) && b.at == q.place)
	{
		arrive(i);
	}
}

// The robot is on the request location. Work starts in the next step.

void Simulation::arrive(int i)
{
	Robot &b = robots[i];
	Request &q = requests[b.request];
	setRequestState(b.request, RQ_IN_PROGRESS, "robot arrived");
	setRobotState(i, RS_PERFORMING, "working");
	b.workLeft = q.duration;
	b.expectRemain = -1;
	q.arriveTime = now;
	logStart("ARRIVE", "robot", b.id, "-", "-");
	logFile << "request = " << q.id << " location = " << env.locs[b.at].id << " response = " << (now - q.reportTime) << endl;
}

void Simulation::doWork(int i)
{
	Robot& b = robots[i];
	b.workLeft--;
	b.workSteps++;

	spend(b, params.batteryPerWork);

	if (b.workLeft > 0)
	{
		return;
	}

	int qi = b.request;
	Request &q = requests[qi];

	if (setRequestState(qi, RQ_COMPLETED, "work finished"))
	{
		q.doneTime = now;
		doneCount++;
	}

	logStart("WORK_DONE", "robot", b.id, "-", "-");
	logFile << "request = " << q.id << " battery = " << b.battery << endl;
	b.request = -1;
	setRobotState(i, RS_AVAILABLE, "request done");
}

void Simulation::rechargeStep(int i)
{
	Robot &b = robots[i];
	int add = params.rechargePerStep;

	if (b.battery + add > b.maxBattery)
	{
		add = b.maxBattery - b.battery;
	}

	b.battery += add;
	b.recharged += add;

	if (b.battery >= b.maxBattery)
	{
		logStart("BATTERY", "robot", b.id, "-", "-");
		logFile << "fully charged" << endl;
		setRobotState(i, RS_AVAILABLE, "fully charged");
	}
}

// robot with nothing to do : recharge if on a working station or\
   head for the nearest working station when its battery is low.

void Simulation::idleStep(int i)
{
	Robot& b = robots[i];

	if (atWorkingStation(b))
	{
		if (b.battery < b.maxBattery)
		{
			setRobotState(i, RS_RECHARGING, "on a working station");
			rechargeStep(i);
		}

		return;
	}

	if (b.battery * 100 >= params.lowBatteryPercent * b.maxBattery)
	{
		return;
	}

	Tree *t = env.chargerTree();
	int d = t->dist[b.at];

	if (d >= NO_ROUTE || b.battery < d * params.batteryPerCost)
	{
		if (!b.warned)
		{
			b.warned = true;
			logStart("BATTERY", "robot", b.id, "-", "-");
			logFile << "low battery (" << b.battery << ") and no working station within reach, staying at location " << env.locs[b.at].id << endl;
		}

		return;
	}

	logStart("BATTERY", "robot", b.id, "-", "-");
	logFile << "low battery (" << b.battery << "), heading for the nearest working station, cost " << d << endl;
	setRobotState(i, RS_RETURNING, "low battery");
	returnTrip(i);
}

// Walking to nearest working station. The road is chosen again at every location,
// so a station that went down is dropped and another one is picked.

void Simulation::returnTrip(int i)
{
	Robot &b = robots[i];

	if (b.roadTo == -1)
	{
		if (atWorkingStation(b))
		{
			setRobotState(i, RS_RECHARGING, "reached station");
			return;
		}
		
		Tree *t = env.chargerTree();
		int d = t->dist[b.at];

		if (d >= NO_ROUTE || b.battery < d * params.batteryPerCost)
		{
			logStart("BATTERY", "robot", b.id, "-", "-");
			logFile << "no working station within reach, staying at location " << env.locs[b.at].id << endl;
			setRobotState(i, RS_AVAILABLE, "no station within reach");
			return;
		}

		startRoad(b, t->pconn[b.at], t->parent[b.at]);
	}

	if (moveAlongRoad(b) && atWorkingStation(b))
	{
		setRobotState(i, RS_RECHARGING, "reached station");
	}
}