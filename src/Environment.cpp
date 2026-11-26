#include "Environment.h"
#include "Constants.h"
#include "Heap.h"

// Tree

Tree::Tree()
{
	version = -1;
	stationVersion = -1;
	dist = 0;
	parent = 0;
	pconn = 0;
}

Tree::~Tree()
{
	delete[] dist;
	delete[] parent;
	delete[] pconn;
}

void Tree::allocate(int n)
{
	if (dist != 0)
	{
		return;
	}

	dist = new int[n + 1];
	parent = new int[n + 1];
	pconn = new int[n + 1];
}

// Environment

Environment::Environment()
{
	cache = 0;
	locs = 0;
	numLocs = 0;
	conns = 0;
	numConns = 0;
	mapVersion = 0;
	stationVersion = 0;
	treeBuilds = 0;
}

Environment::~Environment()
{
	for (int i = 0 ; i < numLocs ; i++)
	{
		AdjNode *node = locs[i].firstRoad;

		while (node != 0)
		{
			AdjNode *nextNode = node->next;
			delete node;
			node = nextNode;
		}

		if (cache != 0)
		{
			delete cache[i];
		}
	}
	delete[] cache;
	delete[] locs;
	delete[] conns;
}

void Environment::reserveLocations(int n)
{
	delete[] locs;
	locs = new Location[n + 1];
	numLocs = 0;
}

void Environment::reserveConnections(int n)
{
	delete[] conns;
	conns = new Connection[n + 1];
	numConns = 0;
}

int Environment::findLocation(int id)
{
	for (int i = 0 ; i < numLocs ; i++)
	{
		if (locs[i].id == id)
		{
			return i;
		}
	}

	return -1;
}

int Environment::findConnection(int id)
{
	for (int i = 0 ; i < numConns ; i++)
	{
		if (conns[i].id == id)
		{
			return i;
		}
	}

	return -1;
}

// Every road is stored twice, once for each end as the map is undirected.\
   New nodes are added at the front of a location's linked list.

void Environment::buildAdjacency()
{
	for (int i = 0 ; i < numLocs ; i++)
	{
		locs[i].firstRoad = 0;
	}

	for (int i = 0 ; i < numConns ; i++)
	{
		int a = findLocation(conns[i].aId);
		int b = findLocation(conns[i].bId);
		conns[i].aIdx = a;
		conns[i].bIdx = b;

		AdjNode* toB = new AdjNode;
		toB->to = b;
		toB->conn = i;
		toB->next = locs[a].firstRoad;
		locs[a].firstRoad = toB;

		AdjNode* toA = new AdjNode;
		toA->to = a;
		toA->conn = i;
		toA->next = locs[b].firstRoad;
		locs[b].firstRoad = toA;
	}

	cache = new Tree * [numLocs + 1];

	for (int i = 0 ; i < numLocs ; i++)
	{
		cache[i] = 0;
	}
}

int Environment::countBlocked()
{
	int total = 0;
	for (int i = 0 ; i < numConns ; i++)
	{
		if (conns[i].blocked)
		{
			total++;
		}
	}

	return total;
}

int Environment::countOpenRoads(int locIdx)
{
	int total = 0;

	for (AdjNode *r = locs[locIdx].firstRoad ; r != 0 ; r = r->next)
	{
		if (!conns[r->conn].blocked)
		{
			total++;
		}
	}
	
	return total;
}

// Dijkstra with a binary heap, over OPEN roads only.

void Environment::growTree(Tree *t, int *sources, int howMany)
{
	t->allocate(numLocs);

	for (int i = 0 ; i < numLocs ; i++)
	{
		t->dist[i] = NO_ROUTE;
		t->parent[i] = -1;
		t->pconn[i] = -1;
	}

	MinHeap heap(2 * numConns + numLocs + 2);

	for (int s = 0 ; s < howMany ; s++)
	{
		t->dist[sources[s]] = 0;
		heap.push(0, sources[s]);
	}

	while (!heap.isEmpty())
	{
		int d;
		int u;
		heap.pop(d, u);

		if (d > t->dist[u])
		{
			continue;
		}

		for (AdjNode *r = locs[u].firstRoad ; r != 0 ; r = r->next)
		{
			if (conns[r->conn].blocked)
			{
				continue;
			}

			int v = r->to;
			int newDist = d + conns[r->conn].cost;

			if (newDist < t->dist[v])
			{
				t->dist[v] = newDist;
				t->parent[v] = u;
				t->pconn[v] = r->conn;
				heap.push(newDist, v);
			}
		}
	}

	treeBuilds++;
}

// Tree grown from one location. It is rebuilt only when a road has changed since it was built.

Tree *Environment::treeFor(int locIdx) 
{
	if (cache[locIdx] == 0)
	{
		cache[locIdx] = new Tree();
	}

	Tree *t = cache[locIdx];

	if (t->version != mapVersion)
	{
		growTree(t, &locIdx, 1);
		t->version = mapVersion;
	}
	return t;
}

// Tree grown from every working charging station.\
   Dist is the distance to the nearest one.

Tree *Environment::chargerTree()
{
	if (charger.version != mapVersion || charger.stationVersion != stationVersion)
	{
		int *sources = new int[numLocs + 1];
		int howMany = 0;
		for (int i = 0 ; i < numLocs ; i++)
		{
			if (locs[i].type == LT_CHARGING && locs[i].stationUp)
			{
				sources[howMany] = i;
				howMany++;
			}
		}

		growTree(&charger, sources, howMany);
		delete[] sources;
		charger.version = mapVersion;
		charger.stationVersion = stationVersion;
	}

	return &charger;
}

bool Environment::shortestPath(int fromId, int toId, int *path, int &length, int &cost)
{
	length = 0;
	cost = 0;
	int from = findLocation(fromId);
	int to = findLocation(toId);
	
	if (from == -1 || to == -1)
	{
		return false;
	}

	Tree *t = treeFor(to);

	if (t->dist[from] >= NO_ROUTE)
	{
		return false;
	}

	cost = t->dist[from];
	int v = from;

	while (v != -1)
	{
		path[length] = v;
		length++;
		v = t->parent[v];
	}

	return true;
}