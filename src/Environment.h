#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "Records.h"

// dist[v]   cost of the best route\
   parent[v] the next location on that route when walking from v towards q\
   pconn[v]  the connection used for that step

class Tree
{
	public :

	int version;
	int stationVersion;
	int *dist;
	int *parent;
	int *pconn;

	Tree();
	~Tree();

	void allocate(int n);
};

class Environment
{
	Tree **cache;
	Tree charger;

	void growTree(Tree *t, int *sources, int howMany);

	public :

	Location *locs;
	int numLocs;

	Connection *conns;
	int numConns;

	int mapVersion;      // goes up whenever a BLOCK or UNBLOCK changes something
	int stationVersion;  // goes up whenever a STATION_DOWN or STATION_UP changes something
	int treeBuilds;      // number of Dijkstra runs

	Environment();
	~Environment();

	void reserveLocations(int n);
	void reserveConnections(int n);
	int findLocation(int id);
	int findConnection(int id);
	void buildAdjacency();

	int countBlocked();
	int countOpenRoads(int locIdx);

	Tree *treeFor(int locIdx);
	Tree *chargerTree();
	bool shortestPath(int fromId, int toId, int *path, int &length, int &cost);
};

#endif