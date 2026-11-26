#ifndef HEAP_H
#define HEAP_H

#include "Records.h"

// Min heap of (distance, location index) pairs, with equal distances ordered by location ID.

class MinHeap
{
	int *dist;
	int *node;
	Location *locations;
	int size;
	int capacity;

	bool before(int i, int j);
	void swapItems(int i, int j);

	public :

	MinHeap(int capacity, Location *locationArray);
	~MinHeap();

	bool isEmpty();
	void push(int d, int n);
	void pop(int &d, int &n);
};

// Max-heap of pending requests.\
   higher urgency, then earlier report time, then lower id.\
   Every request remembers its position in the heap (heapPos),\
   so SET_URGENCY can fix one entry without rebuilding the heap.

class PendingHeap
{
	int *items;
	int size;
	Request *requests;

	bool before(int x, int y);
	void put(int pos, int r);
	void moveUp(int i);
	void moveDown(int i);

	public :

	PendingHeap();
	~PendingHeap();

	void init(Request *requestArray, int capacity);
	int getSize();
	void push(int r);
	int pop();
	void update(int r);
};

#endif