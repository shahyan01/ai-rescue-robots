#include <iostream>
#include <fstream>
#include <string>
#include "Simulation.h"
#include "ScenarioLoader.h"
using namespace std;

void showMenu() {
    cout << "\n--- AI Rescue Robots ---\n";
    cout << " 1. Advance one time step\n";
    cout << " 2. Advance N time steps\n";
    cout << " 3. Run to the end\n";
    cout << " 4. Show environment\n";
    cout << " 5. Show robots\n";
    cout << " 6. Show requests\n";
    cout << " 7. Query a route\n";
    cout << " 8. Show statistics\n";
    cout << " 0. Quit\n";
    cout << "Choice: ";
}

void saveStats(Simulation& sim, string logPath) {
    int len = logPath.length();
    if (len > 4 && logPath.substr(len - 4) == ".txt")
        logPath = logPath.substr(0, len - 4);
    logPath = logPath + "_stats.txt";

    ofstream file(logPath.c_str());
    if (!file) {
        cout << "Cannot write statistics file " << logPath << endl;
        return;
    }
    sim.printStats(file);
    file.close();
    cout << "Statistics written to " << logPath << endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cout << "Usage: rescue <scenario file> [auto] [log file]" << endl;
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

    bool autoRun = false;
    if (argc >= 3 && string(argv[2]) == "auto")
        autoRun = true;

    string logPath = "simulation_log.txt";
    if (argc >= 4)
        logPath = argv[3];

    if (!sim.openLog(logPath)) {
        cout << "Cannot write log file " << logPath << endl;
        return 3;
    }
    for (int i = 0; i < warns.total(); i++)
        sim.logLine("WARN", "scenario", 0, "-", "-", warns.msgs[i]);

    cout << "Loaded " << sim.scenarioName << ": "
         << sim.env.locs.size() << " locations, "
         << sim.env.conns.size() << " connections, "
         << sim.robots.size() << " robots, "
         << sim.requests.size() << " requests, "
         << sim.events.size() << " events" << endl;

    if (autoRun) {
        sim.runAll();
        sim.printStats(cout);
        cout << "Log written to " << logPath << endl;
        saveStats(sim, logPath);
        return 0;
    }

    int choice = -1;
    while (choice != 0) {
        showMenu();
        if (!(cin >> choice))
            break;

        if (choice == 1) {
            sim.step();
            cout << "Now at time step " << sim.now;
            if (sim.isFinished())
                cout << " (finished)";
            cout << endl;
        }
        else if (choice == 2) {
            int k;
            cout << "How many steps? ";
            cin >> k;
            for (int i = 0; i < k && !sim.isFinished(); i++)
                sim.step();
            cout << "Now at time step " << sim.now;
            if (sim.isFinished())
                cout << " (finished)";
            cout << endl;
        }
        else if (choice == 3) {
            sim.runAll();
            cout << "Simulation finished at time step " << sim.endStep << endl;
        }
        else if (choice == 4) sim.printEnvironment();
        else if (choice == 5) sim.printRobots();
        else if (choice == 6) sim.printRequests();
        else if (choice == 7) {
            int a, b;
            cout << "From location id and to location id: ";
            cin >> a >> b;
            sim.printRoute(a, b);
        }
        else if (choice == 8) sim.printStats(cout);
    }

    saveStats(sim, logPath);
    cout << "Log written to " << logPath << endl;
    return 0;
}
