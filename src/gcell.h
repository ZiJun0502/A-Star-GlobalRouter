#ifndef GCELL_H
#define GCELL_H

#include <cstddef>
#include <fstream>

class GCell {
public:
    size_t left_edge_capacity;
    size_t bottom_edge_capacity;
    double cost;

    GCell()
        : left_edge_capacity(0), bottom_edge_capacity(0), cost(0) {}
    GCell(size_t left_edge_capacity, size_t bottom_edge_capacity, double cost)
        : left_edge_capacity(left_edge_capacity), bottom_edge_capacity(bottom_edge_capacity), cost(cost) {}
};

#endif // GCELL_H
