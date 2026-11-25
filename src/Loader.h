#ifndef LOADER_H
#define LOADER_H

#include "Simulation.h"

// Reads a scenario file into a Simulation and checks it.\
   Every problem is printed with its line number as soon as it is found.

class Loader
{
	void parseParam(char *words[], int n, int lineNo, Simulation &sim);
	void parseLocation(char *words[], int n, int lineNo, Simulation &sim);
	void parseConnection(char *words[], int n, int lineNo, Simulation &sim);
	void parseRobot(char *words[], int n, int lineNo, Simulation &sim);
	void parseRequest(char *words[], int n, int lineNo, Simulation &sim);
	void parseEvent(char *words[], int n, int lineNo, Simulation &sim);
	void closeSection(int section, int remaining, int declared, int lineNo);
	void sortEvents(Simulation &sim);
	void sortRequests(Simulation &sim);
	void validate(Simulation &sim);
	void warn(const char *text);

	public :

	int errorCount;
	int warningCount;
	bool haveMaxSteps;

	Loader();
	bool load(const char *path, Simulation &sim);
};

#endif