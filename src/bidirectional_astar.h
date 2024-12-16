#ifndef BIDIRECTIONAL_ASTAR_H
#define BIDIRECTIONAL_ASTAR_H

#include <unordered_set>
#include <unordered_map>
#include <set>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "globals.h"
#include "router.h"
#include "route_node.h"

class BidirectionalAStar {
public:
    // Constructor and Destructor
    BidirectionalAStar();
    ~BidirectionalAStar();

    // Public method to find path using bidirectional A*
    std::vector<RouteEdge> find_path(const std::pair<int, int>& start,
                                     const std::pair<int, int>& end);

    // Reset the state of the algorithm
    void reset();

private:
    // Internal state variables for A* algorithm
    std::set<RouteNode*, CompareRouteNode> open_list_start;
    std::set<RouteNode*, CompareRouteNode> open_list_end;
    std::unordered_set<int> open_set_start;
    std::unordered_set<int> open_set_end;
    std::unordered_set<int> closed_set_start;
    std::unordered_set<int> closed_set_end;
    std::unordered_map<int, RouteNode*> allocated_map_start;
    std::unordered_map<int, RouteNode*> allocated_map_end;
    std::unordered_map<int, double> g_map_start;
    std::unordered_map<int, double> g_map_end;
    
    // Helper methods for A* search
    void expand(RouteNode* current, 
                      std::set<RouteNode*, CompareRouteNode>& open_list, 
                      std::unordered_set<int>& open_set, 
                      std::unordered_set<int>& closed_set, 
                      std::unordered_map<int, RouteNode*>& allocated_map, 
                      std::unordered_map<int, double>& g_map, 
                      int target_x, int target_y, double& min_g, RouteNode **min_node,
                      std::set<RouteNode*, CompareRouteNode>& other_open_list,
                      std::unordered_set<int>& other_open_set, 
                      std::unordered_set<int>& other_closed_set,
                      std::unordered_map<int, RouteNode*>& other_allocated_map,
                      std::unordered_map<int, double>& other_g_map);
    double calculate_heuristic(int x1, int y1, int x2, int y2);
    double calculate_g_cost(int from_x, int from_y, int from_layer, int to_x, int to_y, int to_layer);
    bool is_valid_move(int x, int y, int layer);
    int grid_hash(int x, int y, int layer);
    std::vector<RouteEdge> reconstruct_path(RouteNode* node, int start_x, int start_y, int layer, bool end);
    int get_direction(int x1, int y1, int x2, int y2);
    int get_opposite_direction(int dir);

    // Configuration or additional parameters (e.g., grid size, scaling)
    double h_scale = 1.0; // Heuristic scale
    unsigned long long max_nodes_allowed;
    unsigned long long traversed_nodes;
    unsigned long long num_nodes_per_ms;
    double max_run_time_per_bump;
    double remaining_time_ms;
};

#endif // BIDIRECTIONAL_ASTAR_H
