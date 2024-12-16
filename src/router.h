#ifndef ROUTER_H
#define ROUTER_H

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cmath>
#include <algorithm>
#include <float.h>
#include <chrono>

#include "globals.h"
#include "bidirectional_astar.h"
#include "route_node.h"

class Router {
public:
    Router();
    
    // Route all nets between chips
    bool route_nets();
    
    // Output routing results
    void output_routing_results(std::ofstream& output_file);
    
    // A* path finding
    std::vector<RouteEdge> find_path(const std::pair<int, int>& start, 
                                     const std::pair<int, int>& end);
    void expand_frontier(RouteNode* current, 
                             std::set<RouteNode*, CompareRouteNode>& open_list, 
                             std::unordered_set<int>& closed_set, 
                             std::unordered_map<int, RouteNode*>& allocated_map, 
                             std::unordered_map<int, double>& g_map, 
                             int target_x, int target_y,
                             std::set<RouteNode*, CompareRouteNode>& other_open_list,
                             std::unordered_set<int>& other_closed_set,
                             std::unordered_map<int, RouteNode*>& other_allocated_map,
                             std::unordered_map<int, double>& other_g_map);
    
    // Cost calculation methods
    double calculate_cost(const std::vector<std::vector<RouteEdge>>& routed_net);
    double calculate_wire_length_cost(const std::vector<RouteEdge>& path);
    double calculate_overflow_cost();
    double calculate_gcell_cost(const std::vector<RouteEdge>& path);
    double calculate_via_cost(const std::vector<RouteEdge>& path);
    
    // Grid-related methods

    std::vector<RouteEdge> reconstruct_path(RouteNode* node, int start_x, int start_y);
    bool is_valid_move(int x, int y, int layer);
    int get_direction(int from_x, int from_y, int to_x, int to_y);
    int get_opposite_direction(int dir);
    int grid_hash(int x, int y, int layer);
    // Heuristic calculation
    int calculate_heuristic(int x1, int y1, int x2, int y2);
    double calculate_g_cost(int from_x, int from_y, int from_layer, int to_x, int to_y, int to_layer);
    double recorded_total_cost;
    std::vector<double> recorded_costs;
    // Tracking routed nets and their edges
    std::vector<std::vector<RouteEdge>> routed_nets;
    // Grid tracking for overflow calculation
    static std::array<std::vector<std::vector<int>>, 4> layer_net_count;
};

#endif // ROUTER_H
