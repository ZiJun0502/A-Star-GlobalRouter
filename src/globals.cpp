#include "globals.h"

double alpha, beta, _gamma, delta;
double via_cost;
size_t routing_area_x, routing_area_y, routing_area_width, routing_area_height;
std::array<Chip, 2> chips;
size_t grid_width, grid_height;
size_t routing_area_gwidth, routing_area_gheight;
std::vector<std::pair<size_t, size_t>> edge_capacities; 
std::vector<std::vector<double>> layer1_costs;
std::vector<std::vector<double>> layer2_costs;
std::vector<std::vector<GCell>> layer1_gcells, layer2_gcells;
std::string output_filename;

std::ostream& operator<<(std::ostream& os, const Chip& chip) {
    os << "Chip(x=" << chip.x << ", y=" << chip.y 
       << ", width=" << chip.width << ", height=" << chip.height
       << ", bumps=[";
    for (size_t i = 0; i < chip.bumps.size(); ++i) {
        os << "(" << chip.bumps[i].first << ", " << chip.bumps[i].second << ")";
        if (i != chip.bumps.size() - 1) os << ", ";
    }
    os << "])";
    return os;
}

void print_globals() {
    std::cout << "Alpha: " << alpha << ", Beta: " << beta
              << ", Gamma: " << _gamma << ", Delta: " << delta << std::endl;
    std::cout << "Routing Area Coordinate: (" << routing_area_x << ", " << routing_area_y
              << ") - Width: " << routing_area_width
              << ", Height: " << routing_area_height << std::endl;
    std::cout << "Grid: " << grid_width << "x" << grid_height << std::endl;
    std::cout << chips[0] << std::endl << chips[1] << std::endl;
    std::cout << "Edge Capacities: " << edge_capacities.size() << " entries." << std::endl;

    std::cout << "Layer1: " << std::endl;
    for (size_t i = 0 ; i < routing_area_gheight ; i++) {
        for (size_t j = 0 ; j < routing_area_gwidth ; j++) {
            // std::cout << layer1_gcells[i][j].left_edge_capacity << ' ';
            printf("(%lu, %lu, %f), ", layer1_gcells[i][j].left_edge_capacity, layer1_gcells[i][j].bottom_edge_capacity, layer1_gcells[i][j].cost);
        }
        std::cout << '\n';
    }
    std::cout << "Layer2: " << std::endl;
    for (size_t i = 0 ; i < routing_area_gheight ; i++) {
        for (size_t j = 0 ; j < routing_area_gwidth ; j++) {
            printf("(%lu, %lu, %f), ", layer2_gcells[i][j].left_edge_capacity, layer2_gcells[i][j].bottom_edge_capacity, layer2_gcells[i][j].cost);
        }
        std::cout << '\n';
    }
}