#ifndef SCENARIOLOADER_H
#define SCENARIOLOADER_H

#include "Simulation.h"

// Holds error or warning messages, each with its line number where there is one.
// There is no limit on how many are kept.
class ErrorList {
public:
    DynArray<string> msgs;

    void add(string text);
    int total() const;
    void print(string title) const;
};

// Reads a scenario file into sim. Returns false (and fills errs) when the file cannot
// be opened or contains invalid or inconsistent data. Warnings never stop the run.
bool loadScenario(string path, Simulation& sim, ErrorList& errs, ErrorList& warns);

#endif
