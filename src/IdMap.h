#ifndef IDMAP_H
#define IDMAP_H

class IdMap
{
    int *slot;
    int size;

public :

    static const int MAX_ID = 1000000;

    IdMap();
    ~IdMap();

    // ids[i] is the id of the i-th record. badId then holds the offending id.

    bool build(const int *ids, int n, int &badId);

    int find(int id) const;
};

#endif