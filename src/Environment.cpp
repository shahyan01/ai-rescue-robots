#include "Environment.h"
#include "Heap.h"

Tree::Tree() {
    version = -1;
    stationVersion = -1;
    dist = 0;
    parent = 0;
    pconn = 0;
}

Tree::~Tree() {
    delete[] dist;
    delete[] parent;
    delete[] pconn;
}

Environment::Environment() {
    cache = 0;
    mapVersion = 0;
    stationVersion = 0;
    treeBuilds = 0;
}

Environment::~Environment() {
    if (cache) {
        for (int i = 0; i < locs.size(); i++)
            delete cache[i];
        delete[] cache;
    }
}

int Environment::locIndex(int id) const {
    return locMap.find(id);
}

int Environment::connIndex(int id) const {
    return connMap.find(id);
}

bool Environment::indexLocations(int& badId) {
    int n = locs.size();
    int* ids = new int[n + 1];
    for (int i = 0; i < n; i++)
        ids[i] = locs[i].id;
    bool ok = locMap.build(ids, n, badId);
    delete[] ids;
    return ok;
}

bool Environment::indexConnections(int& badId) {
    int n = conns.size();
    int* ids = new int[n + 1];
    for (int i = 0; i < n; i++)
        ids[i] = conns[i].id;
    bool ok = connMap.build(ids, n, badId);
    delete[] ids;
    return ok;
}

void Environment::buildAdjacency() {
    adj.clear();
    for (int i = 0; i < locs.size(); i++)
        locs[i].adjHead = -1;

    for (int i = 0; i < conns.size(); i++) {
        int a = locIndex(conns[i].aId);
        int b = locIndex(conns[i].bId);
        conns[i].aIdx = a;
        conns[i].bIdx = b;

        AdjNode toB;
        toB.conn = i;
        toB.to = b;
        toB.next = locs[a].adjHead;
        adj.push(toB);
        locs[a].adjHead = adj.size() - 1;

        AdjNode toA;
        toA.conn = i;
        toA.to = a;
        toA.next = locs[b].adjHead;
        adj.push(toA);
        locs[b].adjHead = adj.size() - 1;
    }

    cache = new Tree*[locs.size() + 1];
    for (int i = 0; i < locs.size(); i++)
        cache[i] = 0;
}

int Environment::countBlocked() const {
    int c = 0;
    for (int i = 0; i < conns.size(); i++)
        if (conns[i].blocked)
            c++;
    return c;
}

int Environment::countOpenEdges(int locIdx) const {
    int c = 0;
    for (int e = locs[locIdx].adjHead; e != -1; e = adj[e].next)
        if (!conns[adj[e].conn].blocked)
            c++;
    return c;
}

void Environment::allocate(Tree& t) {
    if (t.dist)
        return;
    int n = locs.size();
    t.dist = new int[n + 1];
    t.parent = new int[n + 1];
    t.pconn = new int[n + 1];
}

// Dijkstra with a binary heap over OPEN connections only.
// A distance is replaced only when strictly smaller, and heap ties go to the lower
// location index, so the tree is the same on every run.
void Environment::grow(Tree& t, const int* sources, int count) {
    allocate(t);
    int n = locs.size();
    for (int i = 0; i < n; i++) {
        t.dist[i] = INF_COST;
        t.parent[i] = -1;
        t.pconn[i] = -1;
    }

    MinHeap heap;
    for (int s = 0; s < count; s++) {
        t.dist[sources[s]] = 0;
        heap.push(0, sources[s]);
    }

    while (!heap.empty()) {
        HeapItem it = heap.pop();
        int u = it.node;
        if (it.d > t.dist[u])
            continue;
        for (int e = locs[u].adjHead; e != -1; e = adj[e].next) {
            const Connection& c = conns[adj[e].conn];
            if (c.blocked)
                continue;
            int v = adj[e].to;
            int nd = it.d + c.cost;
            if (nd < t.dist[v]) {
                t.dist[v] = nd;
                t.parent[v] = u;
                t.pconn[v] = adj[e].conn;
                heap.push(nd, v);
            }
        }
    }
    treeBuilds++;
}

const Tree& Environment::treeFor(int locIdx) {
    if (!cache[locIdx])
        cache[locIdx] = new Tree();
    Tree& t = *cache[locIdx];
    if (t.version != mapVersion) {
        grow(t, &locIdx, 1);
        t.version = mapVersion;
    }
    return t;
}

const Tree& Environment::chargerTree() {
    if (charger.version != mapVersion || charger.stationVersion != stationVersion) {
        int n = locs.size();
        int* src = new int[n + 1];
        int count = 0;
        for (int i = 0; i < n; i++)
            if (locs[i].type == LT_CHARGING && locs[i].stationUp) {
                src[count] = i;
                count++;
            }
        grow(charger, src, count);
        delete[] src;
        charger.version = mapVersion;
        charger.stationVersion = stationVersion;
    }
    return charger;
}

bool Environment::shortestPath(int fromId, int toId, int* path, int& len, int& cost) {
    int from = locIndex(fromId);
    int to = locIndex(toId);
    len = 0;
    cost = 0;
    if (from == -1 || to == -1)
        return false;

    const Tree& t = treeFor(to);
    if (t.dist[from] >= INF_COST)
        return false;

    cost = t.dist[from];
    int v = from;
    while (v != -1) {
        path[len] = v;
        len++;
        v = t.parent[v];
    }
    return true;
}
