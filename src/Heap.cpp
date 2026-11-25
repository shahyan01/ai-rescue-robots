#include "Heap.h"

// ---------- MinHeap ----------

MinHeap::MinHeap(int cap) {
	capacity = cap;
	size = 0;
	dist = new int[capacity];
	node = new int[capacity];
}

MinHeap::~MinHeap() {
	delete[] dist;
	delete[] node;
}

bool MinHeap::isEmpty() {
	return size == 0;
}

bool MinHeap::before(int i, int j) {
	if (dist[i] != dist[j])
		return dist[i] < dist[j];
	return node[i] < node[j];
}

void MinHeap::swapItems(int i, int j) {
	int d = dist[i];
	int n = node[i];
	dist[i] = dist[j];
	node[i] = node[j];
	dist[j] = d;
	node[j] = n;
}

void MinHeap::push(int d, int n) {
	dist[size] = d;
	node[size] = n;
	int i = size;
	size++;
	while (i > 0) {
		int parent = (i - 1) / 2;
		if (!before(i, parent))
			break;
		swapItems(i, parent);
		i = parent;
	}
}

void MinHeap::pop(int& d, int& n) {
	d = dist[0];
	n = node[0];
	size--;
	dist[0] = dist[size];
	node[0] = node[size];
	int i = 0;
	while (true) {
		int leftChild = 2 * i + 1;
		int rightChild = leftChild + 1;
		int smallest = i;
		if (leftChild < size && before(leftChild, smallest))
			smallest = leftChild;
		if (rightChild < size && before(rightChild, smallest))
			smallest = rightChild;
		if (smallest == i)
			break;
		swapItems(i, smallest);
		i = smallest;
	}
}

// ---------- PendingHeap ----------

PendingHeap::PendingHeap() {
	items = 0;
	size = 0;
	requests = 0;
}

PendingHeap::~PendingHeap() {
	delete[] items;
}

void PendingHeap::init(Request* requestArray, int capacity) {
	requests = requestArray;
	items = new int[capacity + 1];
	size = 0;
}

int PendingHeap::getSize() {
	return size;
}

bool PendingHeap::before(int x, int y) {
	if (requests[x].urgency != requests[y].urgency)
		return requests[x].urgency > requests[y].urgency;
	if (requests[x].reportTime != requests[y].reportTime)
		return requests[x].reportTime < requests[y].reportTime;
	return requests[x].id < requests[y].id;
}

void PendingHeap::put(int pos, int r) {
	items[pos] = r;
	requests[r].heapPos = pos;
}

void PendingHeap::moveUp(int i) {
	while (i > 0) {
		int parent = (i - 1) / 2;
		if (!before(items[i], items[parent]))
			break;
		int a = items[i];
		int b = items[parent];
		put(i, b);
		put(parent, a);
		i = parent;
	}
}

void PendingHeap::moveDown(int i) {
	while (true) {
		int leftChild = 2 * i + 1;
		int rightChild = leftChild + 1;
		int best = i;
		if (leftChild < size && before(items[leftChild], items[best]))
			best = leftChild;
		if (rightChild < size && before(items[rightChild], items[best]))
			best = rightChild;
		if (best == i)
			break;
		int a = items[i];
		int b = items[best];
		put(i, b);
		put(best, a);
		i = best;
	}
}

void PendingHeap::push(int r) {
	put(size, r);
	size++;
	moveUp(size - 1);
}

int PendingHeap::pop() {
	int top = items[0];
	requests[top].heapPos = -1;
	size--;
	if (size > 0) {
		put(0, items[size]);
		moveDown(0);
	}
	return top;
}

// Call after the urgency of a request that is inside the heap has changed.
void PendingHeap::update(int r) {
	int pos = requests[r].heapPos;
	if (pos < 0)
		return;
	moveUp(pos);
	moveDown(requests[r].heapPos);
}
