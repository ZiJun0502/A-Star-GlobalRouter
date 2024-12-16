#include "bidirectional_astar.h"

BidirectionalAStar::BidirectionalAStar() {}

BidirectionalAStar::~BidirectionalAStar() {}

void BidirectionalAStar::reset() {
    for (auto& p : allocated_map_start) {
        delete p.second;
    }
    for (auto& p : allocated_map_end) {
        delete p.second;
    }
    closed_set_start.clear();
    closed_set_end.clear();
    allocated_map_start.clear();
    allocated_map_end.clear();
    g_map_start.clear();
    g_map_end.clear();
    open_list_start.clear();
    open_list_end.clear();
    open_set_start.clear();
    open_set_end.clear();
    traversed_nodes = 0;
}

std::vector<RouteEdge> BidirectionalAStar::find_path(const std::pair<int, int>& start,
                                                     const std::pair<int, int>& end) {
    int start_x = start.first / grid_width;
    int start_y = start.second / grid_height;
    int end_x = end.first / grid_width;
    int end_y = end.second / grid_height;

    // Set up initial state
    reset();
    max_nodes_allowed = std::max(max_run_time_per_bump * num_nodes_per_ms, 4.0 * (routing_area_gheight + routing_area_gwidth));

    // Initialize start node
    RouteNode* start_node = new RouteNode(start_x, start_y, 0, 0, 0, calculate_heuristic(start_x, start_y, end_x, end_y));
    allocated_map_start[grid_hash(start_x, start_y, 0)] = start_node;
    open_list_start.insert(start_node);
    open_set_start.insert(grid_hash(start_x, start_y, 0));

    // Initialize end node
    RouteNode* end_node = new RouteNode(end_x, end_y, 0, 0, 0, calculate_heuristic(end_x, end_y, start_x, start_y));
    allocated_map_end[grid_hash(end_x, end_y, 0)] = end_node;
    open_list_end.insert(end_node);
    open_set_end.insert(grid_hash(end_x, end_y, 0));

    bool path_found = false;
    double min_g = DBL_MAX;
    RouteNode* min_node = nullptr;

    while (!path_found) {
        // Perform bidirectional search here
        // Start search direction
        if (!open_list_start.empty()) {
            start_node = *open_list_start.begin();
            open_list_start.erase(start_node);
            open_set_start.erase(grid_hash(start_node->x, start_node->y, start_node->layer));
            closed_set_start.insert(grid_hash(start_node->x, start_node->y, start_node->layer));
            expand(start_node, open_list_start, open_set_start, closed_set_start, allocated_map_start, g_map_start, end_x, end_y, min_g, &min_node,
                                   open_list_end,   open_set_end,   closed_set_end,   allocated_map_end,   g_map_end);
        }
        // End search direction
        if (!open_list_end.empty()) {
            end_node = *open_list_end.begin();
            open_list_end.erase(end_node);
            open_set_end.erase(grid_hash(end_node->x, end_node->y, end_node->layer));
            closed_set_end.insert(grid_hash(end_node->x, end_node->y, end_node->layer));

            expand(end_node, open_list_end,   open_set_end,   closed_set_end,   allocated_map_end,   g_map_end, end_x, end_y, min_g, &min_node,
                                   open_list_start, open_set_start, closed_set_start, allocated_map_start, g_map_start);
            // if (node != nullptr) {
            //     // printf("Path found from end\n");
            //     break;
            // }
        }
        // printf("min_g: %f, node: %p\n", min_g, min_node);
        if (start_node->g_cost + end_node->g_cost >= min_g)
            break;
    }
    // printf("min_node: %d\n", min_node != nullptr);
    // trace path
    int meet_x = min_node->x, meet_y = min_node->y;
    RouteNode *meet_node_start, *meet_node_end;
    meet_node_start = (allocated_map_start.find(grid_hash(meet_x, meet_y, 0)) == allocated_map_start.end()) ? allocated_map_start[grid_hash(meet_x, meet_y, 1)] : allocated_map_start[grid_hash(meet_x, meet_y, 0)]; 
    meet_node_end = (allocated_map_end.find(grid_hash(meet_x, meet_y, 0)) == allocated_map_end.end()) ? allocated_map_end[grid_hash(meet_x, meet_y, 1)] : allocated_map_end[grid_hash(meet_x, meet_y, 0)]; 
    std::vector<RouteEdge> path_start = reconstruct_path(meet_node_start, start_x, start_y, 0, false);
    std::vector<RouteEdge> path_end = reconstruct_path(meet_node_end, end_x, end_y, 0, true);
    path_end.begin()->is_via = path_start.back().layer != path_end.begin()->layer;
    auto last_edge = path_end.back();
    if (last_edge.layer != 0) {
        path_end.emplace_back(last_edge.end_x, last_edge.end_y, last_edge.end_x, last_edge.end_y, 0, true);
    }
    // std::cout << path_start << '\n';
    // std::cout << path_end << '\n';
    path_start.insert(path_start.end(), path_end.begin(), path_end.end());
    // std::cout << path_start;
    return path_start;
}

void BidirectionalAStar::expand(RouteNode* current, 
                                std::set<RouteNode*, CompareRouteNode>& open_list, 
                                std::unordered_set<int>& open_set, 
                                std::unordered_set<int>& closed_set, 
                                std::unordered_map<int, RouteNode*>& allocated_map, 
                                std::unordered_map<int, double>& g_map, 
                                int target_x, int target_y, double& min_g, RouteNode** min_node,
                                std::set<RouteNode*, CompareRouteNode>& other_open_list,
                                std::unordered_set<int>& other_open_set, 
                                std::unordered_set<int>& other_closed_set,
                                std::unordered_map<int, RouteNode*>& other_allocated_map,
                                std::unordered_map<int, double>& other_g_map) {
    static const int dx[] = {1, -1, 0, 0};
    static const int dy[] = {0, 0, 1, -1};
    for (int i = 0; i < 4; ++i) {
        int new_x = current->x + dx[i];
        int new_y = current->y + dy[i];
        bool target_layer = (dy[i] == 0);
        int neighbor_hash = grid_hash(new_x, new_y, target_layer);

        // Skip invalid moves
        if (!is_valid_move(new_x, new_y, target_layer) || closed_set.find(neighbor_hash) != closed_set.end()) 
            continue;

        // Calculate costs
        double g_cost = current->g_cost + calculate_g_cost(current->x, current->y, current->layer, new_x, new_y, target_layer);
        double h_cost = calculate_heuristic(new_x, new_y, target_x, target_y);
        double f_cost = g_cost + h_cost;

        if (g_map.find(neighbor_hash) == g_map.end() || g_cost < g_map[neighbor_hash]) {
            g_map[neighbor_hash] = g_cost;
            RouteNode* neighbor = new RouteNode(new_x, new_y, target_layer, f_cost, g_cost, h_cost, current);
            RouteNode* neighbor_old = allocated_map[neighbor_hash];
            // Check if already in the open list
            if (open_set.find(neighbor_hash) == open_set.end()) {
                // if the position is not in the open list
                open_set.insert(neighbor_hash);
                if (!neighbor_old) {
                    delete neighbor_old;
                }
            } else {
                // if the position is already in the open list, 
                // we need to modify the g_cost of that entry by re-inserting
                open_list.erase(neighbor_old);
                delete neighbor_old;
            }
            allocated_map[neighbor_hash] = neighbor;
            open_list.insert(neighbor);
            if (other_closed_set.find(neighbor_hash) != other_closed_set.end() && g_cost + other_g_map[neighbor_hash] < min_g) {
                min_g = g_cost + other_g_map[neighbor_hash];
                *min_node = neighbor;
            }
        }
    }
    return;
}
double BidirectionalAStar::calculate_heuristic(int x1, int y1, int x2, int y2) {
    // return 0;
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

double BidirectionalAStar::calculate_g_cost(int from_x, int from_y, int from_layer, int to_x, int to_y, int to_layer) {
    int dir = get_direction(from_x, from_y, to_x, to_y);
    double wire_len = 0.0, move_cost = 0.0, net_via_cost = 0.0, ov_cost = 0.0;

    wire_len = (dir == LEFT || dir == RIGHT) ? grid_width : grid_height;
    move_cost = (from_layer != to_layer) ? (layer_costs[from_layer][from_y][from_x] + layer_costs[to_layer][from_y][from_x]) / 2 
                                                            :layer_costs[from_layer][from_y][from_x];
    net_via_cost = (from_layer != to_layer) * via_cost;
    ov_cost = (Router::layer_net_count[dir][to_y][to_x] > edge_capacities[dir][to_y][to_x]) ? gcell_cost_max : 0;

    double g_cost = alpha * wire_len + beta * ov_cost +  _gamma * move_cost + delta * net_via_cost;
    return g_cost;
}

inline bool BidirectionalAStar::is_valid_move(int x, int y, int layer) {
    // Check grid bounds
    return (x < routing_area_gwidth && x >= 0 && y < routing_area_gheight && y >= 0);
}

inline int BidirectionalAStar::grid_hash(int x, int y, int layer) {
    // x: [1, 1500],
    // y: [1, 1000],
    // layer: [0, 1]
    return x + (y << 11) + (layer << 22);
}

std::vector<RouteEdge> BidirectionalAStar::reconstruct_path(RouteNode* node, int start_x, int start_y, int layer, bool reversed) {
    // reversed:
    //   false: from node to (start_x, start_y)
    //   true:  from (start_x, start_y) to node
    std::vector<RouteEdge> path;
    RouteNode *trace = node;
    int end_x = trace->x, end_y = trace->y;
    bool next_is_via = false;
    while (trace->parent) {
        bool turn = (trace->layer != trace->parent->layer);
        int dir = get_direction(trace->parent->x, trace->parent->y, trace->x, trace->y);
        Router::layer_net_count[dir][trace->y][trace->x]++;
        Router::layer_net_count[get_opposite_direction(dir)][trace->parent->y][trace->parent->x]++;
        // Merge the edges by only pushing the edge when met a turn.
        // printf("Node: (%d, %d, %d)\n", trace->x, trace->y, trace->layer);
        // printf("Parent: (%d, %d, %d)\n", trace->parent->x, trace->parent->y, trace->parent->layer);
        if (turn || (trace->parent->x == start_x && trace->parent->y == start_y)) {
            // printf("Push\n");
            if (reversed) {
                path.emplace_back(end_x, end_y, trace->parent->x, trace->parent->y, trace->layer, next_is_via);
            } else {
                path.emplace_back(trace->parent->x, trace->parent->y, end_x, end_y, trace->layer, trace->layer != trace->parent->layer);
            }
            end_x = trace->parent->x;
            end_y = trace->parent->y;
            next_is_via = true;
        }
        trace = trace->parent;
    }
    if (!reversed) {
        std::reverse(path.begin(), path.end());
    }
    return path;
}

inline int BidirectionalAStar::get_opposite_direction(int dir) {
    switch (dir) {
        case 0:
            return 1;
        case 1:
            return 0;
        case 2:
            return 3;
        case 3:
            return 2;
    }
    return -1;
}
inline int BidirectionalAStar::get_direction(int from_x, int from_y, int to_x, int to_y) {
    // 0: left, 
    // 1: right, 
    // 2: bottom
    // 3: up,
    if (from_x < to_x) {
        return LEFT;
    } else if (from_x > to_x) {
        return RIGHT;
    } else if (from_y < to_y) {
        return BOTTOM;
    } else {
        return UP;
    }
    return -1;
}
