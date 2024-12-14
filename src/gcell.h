#ifndef GCELL_H
#define GCELL_H

#include <cstddef>
#include <fstream>

class GCell {
public:
    int left_edge_capacity;
    int bottom_edge_capacity;
    double cost;

    GCell()
        : left_edge_capacity(0), bottom_edge_capacity(0), cost(0) {}
    GCell(int left_edge_capacity, int bottom_edge_capacity, double cost)
        : left_edge_capacity(left_edge_capacity), bottom_edge_capacity(bottom_edge_capacity), cost(cost) {}
};

#endif // GCELL_H
