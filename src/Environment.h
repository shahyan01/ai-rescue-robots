#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "DynArray.h"
#include "IdMap.h"
#include "Types.h"

const int INF_COST = 1000000000;

struct Location {
    int id;
    string name;
    int type;
    bool stationUp;
    int adjHead;
};

struct Connection {
    int id;
    int aId;
    int bId;
    int aIdx;
    int bIdx;
    int cost;
    bool blocked;
};

// One node of a location's adjacency list. The lists are linked through
// array indices so all nodes live in one dynamic array.
struct AdjNode {
    int conn;
    int to;
    int next;
};

// A shortest-path tree. For a tree grown from location q:
//   dist[v]   cost of the best route between v and q over open roads
//   parent[v] the next location on that route when walking from v towards q
//   pconn[v]  the connection used for that step
// version tells which map_version the tree was built for.
struct Tree {
    int version;
    int stationVersion;
    int* dist;
    int* parent;
    int* pconn;

    Tree();
    ~Tree();
};

class Environment {
private:
    IdMap locMap;
    IdMap connMap;
    Tree** cache;       // one slot per location, filled when first needed
    Tree charger;       // grown from every working charging station at once

    void allocate(Tree& t);
    void grow(Tree& t, const int* sources, int count);

public:
    DynArray<Location> locs;
    DynArray<Connection> conns;
    DynArray<AdjNode> adj;
    int mapVersion;      // goes up on every BLOCK or UNBLOCK that changes something
    int stationVersion;  // goes up on every STATION_DOWN or STATION_UP that changes something
    int treeBuilds;      // how many searches have been run (for the performance report)

    Environment();
    ~Environment();

    bool indexLocations(int& badId);
    bool indexConnections(int& badId);
    void buildAdjacency();

    int locIndex(int id) const;
    int connIndex(int id) const;

    int countBlocked() const;
    int countOpenEdges(int locIdx) const;

    // Shortest-path tree grown from one location. Rebuilt only when the map changed.
    const Tree& treeFor(int locIdx);

    // Tree grown from all working charging stations. dist is the distance to the nearest one.
    const Tree& chargerTree();

    // Returns false when no route exists. path receives location indices
    // (from first) and must hold locs.size() entries.
    bool shortestPath(int fromId, int toId, int* path, int& len, int& cost);
};

#endif
