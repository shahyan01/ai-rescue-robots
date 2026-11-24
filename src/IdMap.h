#ifndef IDMAP_H
#define IDMAP_H

// Maps an external id (as written in the scenario file) to its array index.
// Lookup is a single array access, so it is O(1).
class IdMap {
private:
	int* slot;
	int size;

	IdMap(const IdMap&);
	IdMap& operator=(const IdMap&);

public:
	static const int MAX_ID = 1000000;

	IdMap() : slot(0), size(0) {}
	~IdMap() { delete[] slot; }

	// ids[i] is the id of the i-th record. Returns false if an id is
	// negative, too large, or repeated; badId then holds the offending id.
	bool build(const int* ids, int n, int& badId) {
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

	int find(int id) const {
		if (id < 0 || id >= size)
			return -1;
		return slot[id];
	}
};

#endif
