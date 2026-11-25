#include <iostream>
#include "Simulation.h"
#include "Loader.h"
#include "Utils.h"

using namespace std;

void showMenu() {
	cout << endl << "--- AI Rescue Robots ---" << endl;
	cout << " 1. Advance one time step" << endl;
	cout << " 2. Advance N time steps" << endl;
	cout << " 3. Run to the end" << endl;
	cout << " 4. Show environment" << endl;
	cout << " 5. Show robots" << endl;
	cout << " 6. Show requests" << endl;
	cout << " 7. Query a route" << endl;
	cout << " 8. Show statistics" << endl;
	cout << " 0. Quit" << endl;
	cout << "Choice: ";
}

void saveStats(Simulation& sim, const char* statsPath) {
	ofstream out(statsPath);
	if (!out.is_open()) {
		cout << "Cannot write statistics file '" << statsPath << "'" << endl;
		return;
	}
	sim.printStats(out);
	out.close();
	cout << "Statistics written to " << statsPath << endl;
}

// Usage: rescue <scenario file> [auto] [log file] [statistics file]
int main(int argc, char* argv[]) {
	if (argc < 2) {
		cout << "Usage: rescue <scenario file> [auto] [log file] [statistics file]" << endl;
		return 1;
	}

	Simulation sim;
	Loader loader;
	cout << "Loading " << argv[1] << endl;
	if (!loader.load(argv[1], sim)) {
		cout << "Scenario is invalid: " << loader.errorCount << " problem(s) found" << endl;
		return 2;
	}

	bool autoRun = (argc >= 3 && isSame(argv[2], "auto"));
	const char* logPath = "simulation_log.txt";
	const char* statsPath = "statistics.txt";
	if (argc >= 4)
		logPath = argv[3];
	if (argc >= 5)
		statsPath = argv[4];

	if (!sim.openLog(logPath)) {
		cout << "Cannot write log file '" << logPath << "'" << endl;
		return 3;
	}

	cout << "Loaded '" << sim.scenarioName << "': " << sim.env.numLocs << " locations, "
		<< sim.env.numConns << " connections, " << sim.numRobots << " robots, "
		<< sim.numRequests << " requests, " << sim.numEvents << " events" << endl;

	if (autoRun) {
		sim.runAll();
		sim.printStats(cout);
		cout << "Log written to " << logPath << endl;
		saveStats(sim, statsPath);
		return 0;
	}

	int choice = -1;
	while (choice != 0) {
		showMenu();
		cin >> choice;
		if (cin.fail())
			break;
		if (choice == 1) {
			sim.step();
			cout << "Now at time step " << sim.now << (sim.isFinished() ? " (finished)" : "") << endl;
		} else if (choice == 2) {
			int howMany;
			cout << "How many steps? ";
			cin >> howMany;
			for (int i = 0; i < howMany && !sim.isFinished(); i++)
				sim.step();
			cout << "Now at time step " << sim.now << (sim.isFinished() ? " (finished)" : "") << endl;
		} else if (choice == 3) {
			sim.runAll();
			cout << "Simulation finished at time step " << sim.endStep << endl;
		} else if (choice == 4) {
			sim.printEnvironment();
		} else if (choice == 5) {
			sim.printRobots();
		} else if (choice == 6) {
			sim.printRequests();
		} else if (choice == 7) {
			int fromId;
			int toId;
			cout << "From location id and to location id: ";
			cin >> fromId >> toId;
			sim.printRoute(fromId, toId);
		} else if (choice == 8) {
			sim.printStats(cout);
		}
	}
	saveStats(sim, statsPath);
	cout << "Log written to " << logPath << endl;
	return 0;
}
