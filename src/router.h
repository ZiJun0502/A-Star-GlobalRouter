#ifndef ROUTER_H
#define ROUTER_H

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cmath>
#include <algorithm>

#include <boost/functional/hash.hpp>

#include "globals.h"
#include "hash_tuple.h"


struct RouteNode {
    size_t x;
    size_t y;
    size_t layer; // 0 for M1, 1 for M2
    double f_cost; // Total estimated cost
    double g_cost; // Cost from start
    double h_cost; // Heuristic cost to goal
    RouteNode* parent;

    RouteNode(size_t x, size_t y, size_t layer, double f = 0, double g = 0, double h = 0, RouteNode* p = nullptr)
        : x(x), y(y), layer(layer), f_cost(f), g_cost(g), h_cost(h), parent(p) {}

    // Comparison for priority queue
    bool operator>(const RouteNode& other) const {
        return f_cost > other.f_cost;
    }
};
struct RouteNodeComparator {
    bool operator()(const RouteNode& lhs, const RouteNode& rhs) const {
        if (lhs.x != rhs.x) return lhs.x < rhs.x;
        if (lhs.y != rhs.y) return lhs.y < rhs.y;
        return lhs.layer < rhs.layer;
    }
};

struct RouteEdge {
    size_t start_x, start_y, end_x, end_y;
    size_t layer;
    bool is_via;
};

class Router {
public:
    Router();
    
    // Route all nets between chips
    bool route_nets();
    
    // Output routing results
    void output_routing_results(std::ofstream& output_file);

private:
    // A* path finding
    std::vector<RouteEdge> find_path(const std::pair<size_t, size_t>& start, 
                                     const std::pair<size_t, size_t>& end);
    
    // Cost calculation methods
    double calculate_wire_length(const std::vector<RouteEdge>& path);
    double calculate_overflow(const std::vector<RouteEdge>& path);
    double calculate_via_count(const std::vector<RouteEdge>& path);
    
    // Grid-related methods
    void initialize_grid_costs();
    bool is_valid_move(size_t x, size_t y, size_t layer);
    
    // Heuristic calculation
    double calculate_heuristic(size_t x1, size_t y1, size_t x2, size_t y2);
    
    // Tracking routed nets and their edges
    std::vector<std::vector<RouteEdge>> routed_nets;
    
    // Grid tracking for overflow calculation
    std::vector<std::vector<std::vector<size_t>>> layer_net_count;
};

#endif // ROUTER_H
