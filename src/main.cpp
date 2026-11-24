#include <cstdio>
#include <cstring>
#include "Simulation.h"
#include "ScenarioLoader.h"

static void showMenu() {
	printf("\n--- AI Rescue Robots ---\n");
	printf(" 1. Advance one time step\n");
	printf(" 2. Advance N time steps\n");
	printf(" 3. Run to the end\n");
	printf(" 4. Show environment\n");
	printf(" 5. Show robots\n");
	printf(" 6. Show requests\n");
	printf(" 7. Query a route\n");
	printf(" 8. Show statistics\n");
	printf(" 0. Quit\n");
	printf("Choice: ");
}

static void saveStats(const Simulation& sim, const char* logPath) {
	char path[300];
	snprintf(path, sizeof(path), "%s", logPath);
	int len = (int)strlen(path);
	if (len > 4 && strcmp(path + len - 4, ".txt") == 0)
		path[len - 4] = '\0';
	strncat(path, "_stats.txt", sizeof(path) - strlen(path) - 1);
	FILE* f = fopen(path, "w");
	if (!f) {
		printf("Cannot write statistics file '%s'\n", path);
		return;
	}
	sim.printStats(f);
	fclose(f);
	printf("Statistics written to %s\n", path);
}

int main(int argc, char** argv) {
	if (argc < 2) {
		printf("Usage: rescue <scenario file> [auto] [log file]\n");
		return 1;
	}

	Simulation sim;
	ErrorList errs;
	ErrorList warns;
	if (!loadScenario(argv[1], sim, errs, warns)) {
		errs.print("Scenario is invalid, problems");
		return 2;
	}
	if (warns.total() > 0)
		warns.print("Warnings");

	bool autoRun = (argc >= 3 && strcmp(argv[2], "auto") == 0);
	const char* logPath = (argc >= 4) ? argv[3] : "simulation_log.txt";
	if (!sim.openLog(logPath)) {
		printf("Cannot write log file '%s'\n", logPath);
		return 3;
	}
	for (int i = 0; i < warns.total(); i++)
		sim.logLine("WARN", "scenario", 0, "-", "-", "%s", warns.msgs[i].text);

	printf("Loaded '%s': %d locations, %d connections, %d robots, %d requests, %d events\n",
		sim.scenarioName, sim.env.locs.size(), sim.env.conns.size(),
		sim.robots.size(), sim.requests.size(), sim.events.size());

	if (autoRun) {
		sim.runAll();
		sim.printStats(stdout);
		printf("Log written to %s\n", logPath);
		saveStats(sim, logPath);
		return 0;
	}

	int choice = -1;
	while (choice != 0) {
		showMenu();
		if (scanf("%d", &choice) != 1)
			break;
		if (choice == 1) {
			sim.step();
			printf("Now at time step %d%s\n", sim.now, sim.isFinished() ? " (finished)" : "");
		} else if (choice == 2) {
			int k;
			printf("How many steps? ");
			if (scanf("%d", &k) == 1)
				for (int i = 0; i < k && !sim.isFinished(); i++)
					sim.step();
			printf("Now at time step %d%s\n", sim.now, sim.isFinished() ? " (finished)" : "");
		} else if (choice == 3) {
			sim.runAll();
			printf("Simulation finished at time step %d\n", sim.endStep);
		} else if (choice == 4) {
			sim.printEnvironment();
		} else if (choice == 5) {
			sim.printRobots();
		} else if (choice == 6) {
			sim.printRequests();
		} else if (choice == 7) {
			int a, b;
			printf("From location id and to location id: ");
			if (scanf("%d %d", &a, &b) == 2)
				sim.printRoute(a, b);
		} else if (choice == 8) {
			sim.printStats(stdout);
		}
	}
	saveStats(sim, logPath);
	printf("Log written to %s\n", logPath);
	return 0;
}
