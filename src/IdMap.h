#ifndef IDMAP_H
#define IDMAP_H

// Maps an external id (as written in the scenario file) to its array index.
class IdMap {
private:
    int* slot;
    int size;

public:
    static const int MAX_ID = 1000000;

    IdMap();
    ~IdMap();

    // ids[i] is the id of the i-th record. Returns false if an id is
    // negative, too large, or repeated; badId then holds the offending id.
    bool build(const int* ids, int n, int& badId);

    int find(int id) const;
};

#endif
