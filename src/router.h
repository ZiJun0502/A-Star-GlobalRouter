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
    int x;
    int y;
    int layer; // 0 for M1, 1 for M2
    double f_cost; // Total estimated cost
    double g_cost; // Cost from start
    double h_cost; // Heuristic cost to goal
    RouteNode* parent;

    RouteNode(int x, int y, int layer, double f = 0, double g = 0, double h = 0, RouteNode* p = nullptr)
        : x(x), y(y), layer(layer), f_cost(f), g_cost(g), h_cost(h), parent(p) {}

    // Comparison for priority queue
    bool operator>(const RouteNode& other) const {
        return f_cost > other.f_cost;
    }
    bool operator<(const RouteNode& other) const { return f_cost < other.f_cost; }
};
struct CompareRouteNode {
    bool operator()(const RouteNode* lhs, const RouteNode* rhs) const {
        if (lhs->f_cost != rhs->f_cost) 
            return lhs->f_cost < rhs->f_cost;
        if (lhs->x != rhs->x) 
            return lhs->x < rhs->x;
        if (lhs->y != rhs->y) 
            return lhs->y < rhs->y;
        return lhs->layer < rhs->layer;
    }
};

struct RouteEdge {
    int start_x, start_y, end_x, end_y;
    int layer;
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
    std::vector<RouteEdge> find_path(const std::pair<int, int>& start, 
                                     const std::pair<int, int>& end);
    
    // Cost calculation methods
    double calculate_cost(const std::vector<std::vector<RouteEdge>>& routed_net);
    double calculate_wire_length_cost(const std::vector<RouteEdge>& path);
    double calculate_overflow_cost();
    double calculate_gcell_cost(const std::vector<RouteEdge>& path);
    double calculate_via_cost(const std::vector<RouteEdge>& path);
    
    // Grid-related methods
    bool is_valid_move(int x, int y, int layer);
    int get_direction(int from_x, int from_y, int to_x, int to_y);
    int get_opposite_direction(int dir);
    // Heuristic calculation
    double calculate_heuristic(int x1, int y1, int x2, int y2);
    double calculate_g_cost(int from_x, int from_y, int from_layer, int to_x, int to_y, int to_layer);
    double recorded_total_cost;
    std::vector<double> recorded_costs;
    double max_run_time_per_bump;
    double h_scale;
    bool first_net;
    // Tracking routed nets and their edges
    std::vector<std::vector<RouteEdge>> routed_nets;
    // Grid tracking for overflow calculation
    std::array<std::vector<std::vector<int>>, 4> layer_net_count;
};

#endif // ROUTER_H
