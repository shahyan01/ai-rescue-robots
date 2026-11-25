#ifndef UTILS_H
#define UTILS_H

// Small text helpers

bool isSame(const char *a, const char *b);
int textLength(const char *text);
void copyText(char *to, const char *from, int size);
bool toInt(const char *text, int &value);
int splitLine(char *line, char *words[], int maxWords);

// Names for printing and words from the scenario file

const char *locationTypeName(int type);
const char *categoryName(int category);
const char *requestStateName(int state);
const char *robotStateName(int state);
int locationTypeFromWord(const char *word);
int categoryFromWord(const char *word);

#endif