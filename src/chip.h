#ifndef CHIP_H
#define CHIP_H

#include <vector>
#include <utility> 
#include <fstream> 

class Chip {
public:
    int x;
    int y;
    int width;
    int height;
    std::vector<std::pair<int, int>> bumps; // Stores bump coordinates as pairs of (x, y)

    // Constructor
    Chip() {};
    Chip(int _x, int _y, int _width, int _height) {
        x = _x;
        y = _y;
        width = _width;
        height = _height;
        // make it 1-indexed
        bumps.push_back({0, 0});
    }

    void addBump(int b_x, int b_y) {
        bumps.push_back({b_x, b_y});
    }

    void printChipInfo() const;
};
#endif
