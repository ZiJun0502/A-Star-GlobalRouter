#ifndef GLOBALS_H
#define GLOBALS_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <array>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <cassert>

#include "chip.h"
#include "gcell.h"
#define LEFT 0
#define RIGHT 1
#define BOTTOM 2
#define UP 3
// Cost
extern double alpha, beta, _gamma, delta;
extern double via_cost;
extern double gcell_cost_max;
// GridMap
extern size_t routing_area_x, routing_area_y, routing_area_width, routing_area_height;
extern size_t grid_width, grid_height;
extern size_t routing_area_gwidth, routing_area_gheight;
extern std::array<Chip, 2> chips;
// GCell
extern std::array<std::vector<std::vector<size_t>>, 4> edge_capacities; 
// extern std::vector<std::vector<double>> layer1_costs;
// extern std::vector<std::vector<double>> layer2_costs;
extern std::array<std::vector<std::vector<double>>, 2> layer_costs;
extern std::string output_filename;

std::ostream& operator<<(std::ostream& os, const Chip& chip);
void print_globals();

#endif
