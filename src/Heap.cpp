#include "Heap.h"

// ---------- MinHeap ----------

bool MinHeap::before(const HeapItem& x, const HeapItem& y) const {
    return x.d < y.d || (x.d == y.d && x.node < y.node);
}

bool MinHeap::empty() const {
    return a.size() == 0;
}

void MinHeap::push(int d, int node) {
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

HeapItem MinHeap::pop() {
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
            if (l < n && before(a[l], a[m]))
                m = l;
            if (r < n && before(a[r], a[m]))
                m = r;
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

// ---------- PendingHeap ----------

PendingHeap::PendingHeap() {
    reqs = 0;
}

void PendingHeap::init(DynArray<Request>* r) {
    reqs = r;
}

int PendingHeap::size() const {
    return a.size();
}

bool PendingHeap::before(int x, int y) const {
    const Request& p = (*reqs)[x];
    const Request& q = (*reqs)[y];
    if (p.urgency != q.urgency)
        return p.urgency > q.urgency;
    if (p.reportTime != q.reportTime)
        return p.reportTime < q.reportTime;
    return p.id < q.id;
}

void PendingHeap::put(int pos, int r) {
    a[pos] = r;
    (*reqs)[r].heapPos = pos;
}

void PendingHeap::up(int i) {
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

void PendingHeap::down(int i) {
    int n = a.size();
    int r = a[i];
    while (true) {
        int l = 2 * i + 1;
        int rt = l + 1;
        int best = r;
        int bestPos = i;
        if (l < n && before(a[l], best)) {
            best = a[l];
            bestPos = l;
        }
        if (rt < n && before(a[rt], best)) {
            best = a[rt];
            bestPos = rt;
        }
        if (bestPos == i)
            break;
        put(i, best);
        i = bestPos;
    }
    put(i, r);
}

void PendingHeap::push(int r) {
    a.push(r);
    (*reqs)[r].heapPos = a.size() - 1;
    up(a.size() - 1);
}

int PendingHeap::pop() {
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

void PendingHeap::update(int r) {
    int pos = (*reqs)[r].heapPos;
    if (pos < 0)
        return;
    up(pos);
    down((*reqs)[r].heapPos);
}
