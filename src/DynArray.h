#ifndef DYNARRAY_H
#define DYNARRAY_H

// A template has to be fully written in the header, so this is the only
// header that contains code.
template <typename T>
class DynArray {
private:
    T* data;
    int count;
    int capacity;

    DynArray(const DynArray&);
    DynArray& operator=(const DynArray&);

    void grow() {
        int newCap = 8;
        if (capacity > 0)
            newCap = capacity * 2;
        T* newData = new T[newCap];
        for (int i = 0; i < count; i++)
            newData[i] = data[i];
        delete[] data;
        data = newData;
        capacity = newCap;
    }

public:
    DynArray() {
        data = 0;
        count = 0;
        capacity = 0;
    }

    ~DynArray() {
        delete[] data;
    }

    void push(const T& value) {
        if (count == capacity)
            grow();
        data[count] = value;
        count++;
    }

    void pop() {
        if (count > 0)
            count--;
    }

    T& operator[](int i) { return data[i]; }
    const T& operator[](int i) const { return data[i]; }
    int size() const { return count; }
    void clear() { count = 0; }
};

#endif
