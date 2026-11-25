#include "Utils.h"

const char* locationTypeNames[6] = {"JUNCTION", "BUILDING", "HOSPITAL", "DEPOT", "CHARGING", "COMMAND"};
const char* categoryNames[5] = {"MEDICAL", "FIRE", "SUPPLY", "SEARCH", "GENERAL"};
const char* requestStateNames[6] = {"NOT_REPORTED", "PENDING", "ASSIGNED", "IN_PROGRESS", "COMPLETED", "FAILED"};
const char* robotStateNames[6] = {"AVAILABLE", "TRAVELLING", "PERFORMING", "RETURNING", "RECHARGING", "UNAVAILABLE"};

bool isSame(const char* a, const char* b) {
	int i = 0;
	while (a[i] != '\0' && b[i] != '\0') {
		if (a[i] != b[i])
			return false;
		i++;
	}
	return a[i] == b[i];
}

int textLength(const char* text) {
	int n = 0;
	while (text[n] != '\0')
		n++;
	return n;
}

void copyText(char* to, const char* from, int size) {
	int i = 0;
	while (from[i] != '\0' && i < size - 1) {
		to[i] = from[i];
		i++;
	}
	to[i] = '\0';
}

// Reads a whole number (optional sign, digits only, at most 9 digits).
bool toInt(const char* text, int& value) {
	int i = 0;
	bool negative = false;
	if (text[i] == '-' || text[i] == '+') {
		negative = (text[i] == '-');
		i++;
	}
	if (text[i] == '\0')
		return false;
	int digits = 0;
	int result = 0;
	while (text[i] != '\0') {
		if (text[i] < '0' || text[i] > '9')
			return false;
		result = result * 10 + (text[i] - '0');
		digits++;
		if (digits > 9)
			return false;
		i++;
	}
	value = negative ? -result : result;
	return true;
}

// Cuts a line into words by replacing spaces, tabs and line ends with '\0'.
// words[] points into the line. Returns how many words were found (at most maxWords).
int splitLine(char* line, char* words[], int maxWords) {
	int n = 0;
	int i = 0;
	while (line[i] != '\0' && n < maxWords) {
		while (line[i] == ' ' || line[i] == '\t' || line[i] == '\r' || line[i] == '\n')
			i++;
		if (line[i] == '\0')
			break;
		words[n] = &line[i];
		n++;
		while (line[i] != '\0' && line[i] != ' ' && line[i] != '\t' && line[i] != '\r' && line[i] != '\n')
			i++;
		if (line[i] != '\0') {
			line[i] = '\0';
			i++;
		}
	}
	return n;
}

const char* locationTypeName(int type) { return locationTypeNames[type]; }
const char* categoryName(int category) { return categoryNames[category]; }
const char* requestStateName(int state) { return requestStateNames[state]; }
const char* robotStateName(int state) { return robotStateNames[state]; }

int locationTypeFromWord(const char* word) {
	for (int i = 0; i < 6; i++)
		if (isSame(word, locationTypeNames[i]))
			return i;
	return -1;
}

int categoryFromWord(const char* word) {
	for (int i = 0; i < 5; i++)
		if (isSame(word, categoryNames[i]))
			return i;
	return -1;
}
