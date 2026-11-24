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

	static bool before(const HeapItem& x, const HeapItem& y) {
		return x.d < y.d || (x.d == y.d && x.node < y.node);
	}

public:
	bool empty() const { return a.size() == 0; }

	void push(int d, int node) {
		HeapItem h;
		h.d = d;
		h.node = node;
		a.push(h);
		int i = a.size() - 1;
		while (i > 0) {
			int p = (i - 1) / 2;
			if (!before(a[i], a[p]))
				break;
			HeapItem t = a[i];
			a[i] = a[p];
			a[p] = t;
			i = p;
		}
	}

	HeapItem pop() {
		HeapItem top = a[0];
		HeapItem last = a[a.size() - 1];
		a.pop();
		int n = a.size();
		if (n > 0) {
			a[0] = last;
			int i = 0;
			while (true) {
				int l = 2 * i + 1;
				int r = l + 1;
				int m = i;
				if (l < n && before(a[l], a[m])) m = l;
				if (r < n && before(a[r], a[m])) m = r;
				if (m == i)
					break;
				HeapItem t = a[i];
				a[i] = a[m];
				a[m] = t;
				i = m;
			}
		}
		return top;
	}
};

// Max-heap of pending requests (request indices).
// Order: higher urgency, then earlier report time, then lower id.
// Each request remembers its position (heapPos), so SET_URGENCY can fix a single entry.
class PendingHeap {
private:
	DynArray<int> a;
	DynArray<Request>* reqs;

	bool before(int x, int y) const {
		const Request& p = (*reqs)[x];
		const Request& q = (*reqs)[y];
		if (p.urgency != q.urgency)
			return p.urgency > q.urgency;
		if (p.reportTime != q.reportTime)
			return p.reportTime < q.reportTime;
		return p.id < q.id;
	}

	void put(int pos, int r) {
		a[pos] = r;
		(*reqs)[r].heapPos = pos;
	}

	void up(int i) {
		int r = a[i];
		while (i > 0) {
			int p = (i - 1) / 2;
			if (!before(r, a[p]))
				break;
			put(i, a[p]);
			i = p;
		}
		put(i, r);
	}

	void down(int i) {
		int n = a.size();
		int r = a[i];
		while (true) {
			int l = 2 * i + 1;
			int rt = l + 1;
			int best = r;
			int bestPos = i;
			if (l < n && before(a[l], best)) { best = a[l]; bestPos = l; }
			if (rt < n && before(a[rt], best)) { best = a[rt]; bestPos = rt; }
			if (bestPos == i)
				break;
			put(i, best);
			i = bestPos;
		}
		put(i, r);
	}

public:
	PendingHeap() : reqs(0) {}
	void init(DynArray<Request>* r) { reqs = r; }
	int size() const { return a.size(); }

	void push(int r) {
		a.push(r);
		(*reqs)[r].heapPos = a.size() - 1;
		up(a.size() - 1);
	}

	int pop() {
		int top = a[0];
		int last = a[a.size() - 1];
		a.pop();
		(*reqs)[top].heapPos = -1;
		if (a.size() > 0) {
			a[0] = last;
			(*reqs)[last].heapPos = 0;
			down(0);
		}
		return top;
	}

	// Call after the urgency of a request that is in the heap has changed.
	void update(int r) {
		int pos = (*reqs)[r].heapPos;
		if (pos < 0)
			return;
		up(pos);
		down((*reqs)[r].heapPos);
	}
};

#endif
