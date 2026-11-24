#ifndef SCENARIOLOADER_H
#define SCENARIOLOADER_H

#include "Simulation.h"

// Holds error or warning messages, each with its line number where there is one.
// There is no limit on how many are kept.
struct Message {
	char text[200];
};

class ErrorList {
private:
	ErrorList(const ErrorList&);
	ErrorList& operator=(const ErrorList&);
public:
	DynArray<Message> msgs;

	ErrorList() {}
	void add(const char* fmt, ...);
	int total() const { return msgs.size(); }
	void print(const char* title) const;
};

// Reads a scenario file into sim. Returns false (and fills errs) when the file cannot
// be opened or contains invalid or inconsistent data. Warnings never stop the run.
bool loadScenario(const char* path, Simulation& sim, ErrorList& errs, ErrorList& warns);

#endif
