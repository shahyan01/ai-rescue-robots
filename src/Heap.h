#ifndef HEAP_H
#define HEAP_H

#include "DynArray.h"
#include "Entities.h"

// Min-heap of (distance, location index) pairs for Dijkstra.
// Ties on distance go to the smaller location index, so every run repeats exactly.
struct HeapItem {
    int d;
    int node;
};

class MinHeap {
private:
    DynArray<HeapItem> a;

    bool before(const HeapItem& x, const HeapItem& y) const;

public:
    bool empty() const;
    void push(int d, int node);
    HeapItem pop();
};

// Max-heap of pending requests (request indices).
// Order: higher urgency, then earlier report time, then lower id.
// Each request remembers its position (heapPos), so SET_URGENCY can fix a single entry.
class PendingHeap {
private:
    DynArray<int> a;
    DynArray<Request>* reqs;

    bool before(int x, int y) const;
    void put(int pos, int r);
    void up(int i);
    void down(int i);

public:
    PendingHeap();
    void init(DynArray<Request>* r);
    int size() const;
    void push(int r);
    int pop();

    // Call after the urgency of a request that is in the heap has changed.
    void update(int r);
};

#endif
