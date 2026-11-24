#include "IdMap.h"

IdMap::IdMap() {
    slot = 0;
    size = 0;
}

IdMap::~IdMap() {
    delete[] slot;
}

bool IdMap::build(const int* ids, int n, int& badId) {
    delete[] slot;
    slot = 0;
    size = 0;

    int maxId = 0;
    for (int i = 0; i < n; i++) {
        if (ids[i] < 0 || ids[i] > MAX_ID) {
            badId = ids[i];
            return false;
        }
        if (ids[i] > maxId)
            maxId = ids[i];
    }

    size = maxId + 1;
    slot = new int[size];
    for (int i = 0; i < size; i++)
        slot[i] = -1;

    for (int i = 0; i < n; i++) {
        if (slot[ids[i]] != -1) {
            badId = ids[i];
            return false;
        }
        slot[ids[i]] = i;
    }
    return true;
}

int IdMap::find(int id) const {
    if (id < 0 || id >= size)
        return -1;
    return slot[id];
}
