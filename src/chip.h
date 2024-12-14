#ifndef CHIP_H
#define CHIP_H

#include <vector>
#include <utility> 
#include <fstream> 

class Chip {
public:
    size_t x;
    size_t y;
    size_t width;
    size_t height;
    std::vector<std::pair<size_t, size_t>> bumps; // Stores bump coordinates as pairs of (x, y)

    // Constructor
    Chip() {};
    Chip(size_t _x, size_t _y, size_t _width, size_t _height) {
        x = _x;
        y = _y;
        width = _width;
        height = _height;
        // make it 1-indexed
        bumps.push_back({0, 0});
    }

    void addBump(size_t b_x, size_t b_y) {
        bumps.push_back({b_x, b_y});
    }

    void printChipInfo() const;
};
#endif
