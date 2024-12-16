#include "router.h"
#include <chrono>
#include <float.h>

Router::Router() {
    layer_net_count[0] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    layer_net_count[1] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    layer_net_count[2] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    layer_net_count[3] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    
    max_run_time_per_bump = 400000 / (chips[0].bumps.size() - 1);
    num_nodes_per_ms = 500;
    h_scale = 1.0;
    up = 2.0, down = 0.8;
    rescaled = false;
}


bool Router::route_nets() {
    remaining_time_ms = 400000;
    // Route from each bump of chip[0] to each bump of chip[1]
    for (int i = 1; i < chips[0].bumps.size(); ++i) {
        max_run_time_per_bump = remaining_time_ms / (chips[0].bumps.size() - i);
        std::pair<int, int> start = {chips[0].x + chips[0].bumps[i].first, chips[0].y + chips[0].bumps[i].second};
        std::pair<int, int> end = {chips[1].x + chips[1].bumps[i].first, chips[1].y + chips[1].bumps[i].second};
        
        // std::cout << "Finding path for bump " << i << '\n'; 
        std::vector<RouteEdge> net_path = find_path(start, end);
        if (!net_path.empty()) {
            routed_nets.push_back(net_path);
        } else {
            // Path finding failed
            return false;
        }
    }
    return true;
}

std::vector<RouteEdge> Router::find_path(const std::pair<int, int>& start, 
                                         const std::pair<int, int>& end) {
    // Convert start and end to grid coordinates
    int start_x = (start.first / grid_width);
    int start_y = (start.second / grid_height);
    int end_x = (end.first / grid_width);
    int end_y = (end.second / grid_height);

    static const int dx[] = {1, -1, 0, 0};
    static const int dy[] = {0, 0, 1, -1};
    // around 500 nodes per ms for 1500*1000
    // unsigned long long traversed_count = 0;
    unsigned long long expected_traversed_count = 2 * routing_area_gheight * routing_area_gwidth;
    unsigned long long max_nodes_allowed = std::max(max_run_time_per_bump * num_nodes_per_ms, (double) 4 * routing_area_gheight + routing_area_gwidth);
    unsigned long long traversed_nodes = 0;

    bool path_found = false;
    bool up_scaled = false, down_scaled = false;
    std::vector<RouteEdge> path;
    auto start_ = std::chrono::steady_clock::now();
    auto total_start = std::chrono::steady_clock::now();
    while (!path_found) {
        start_ = std::chrono::steady_clock::now();
        std::set<RouteNode*, CompareRouteNode> open_list;
        std::unordered_set<int> open_set;
        std::unordered_set<int> closed_set;
        
        // std::vector<std::vector<std::vector<RouteNode*>>> allocated_table(routing_area_gheight, std::vector<std::vector<RouteNode*>>(routing_area_gwidth, std::vector<RouteNode*>(2, nullptr)));
        // std::vector<std::vector<std::vector<double>>> g_table(routing_area_gheight, std::vector<std::vector<double>>(routing_area_gwidth, std::vector<double>(2, DBL_MAX)));

        std::unordered_map<int, RouteNode*> allocated_map;
        std::unordered_map<int, double> g_map;

        g_map[grid_hash(start_x, start_y, 0)]= 0;
        RouteNode* start_node = new RouteNode(start_x, start_y, 0, 0, 0, 
            calculate_heuristic(start_x, start_y, end_x, end_y));
        allocated_map[grid_hash(start_x, start_y, 0)]= start_node;

        open_list.insert(start_node);
        open_set.insert(grid_hash(start_x, start_y, 0));

        traversed_nodes = 0;
        
        bool added = 0;
        while (!open_list.empty()) {
            RouteNode* current = *open_list.begin();
            int current_hash = grid_hash(current->x, current->y, current->layer);
            open_list.erase(current);
            open_set.erase(current_hash);
            traversed_nodes++;
            if (traversed_nodes > max_nodes_allowed) {
                up = std::max(1.0 + (up - 1.0) * 0.9, 1.4);
                rescaled = true;
                h_scale *= up;  // Increase h_scale to prioritize heuristic more
                break;
            }
            // target bump reached
            if (current->x == end_x && current->y == end_y) {
                if (rescaled && traversed_nodes < max_nodes_allowed * 0.1) {
                    // Continue scaling down
                    down = std::min(1.0 - (1.0 - down) * 0.9, 0.95);
                    h_scale *= down;
                }
                // Reconstruct path
                path = reconstruct_path(current, start_x, start_y);
                path_found = true;
                break;
            }

            // Mark as visited
            closed_set.insert(current_hash);

            for (int i = 0; i < 4; ++i) {
                int new_x = current->x + dx[i];
                int new_y = current->y + dy[i];

                bool target_layer = (dy[i] == 0);
                int neighbor_hash = grid_hash(new_x, new_y, target_layer);
                // Check if move is valid or is already in closed set
                if (!is_valid_move(new_x, new_y, target_layer) || closed_set.find(neighbor_hash) != closed_set.end()) 
                    continue;
                
                // Calculate cost
                double g_cost = g_map[current_hash] + calculate_g_cost(current->x, current->y, current->layer, new_x, new_y, target_layer);
                double h_cost = h_scale * calculate_heuristic(new_x, new_y, end_x, end_y);
                double f_cost = g_cost + h_cost;
                // Add to open list only if this neighbor:
                // 1. haven't been seen or
                // 2. seen but have a lower g_cost;
                if (g_map.find(neighbor_hash) == g_map.end() || g_cost < g_map[neighbor_hash]) {
                    g_map[neighbor_hash] = g_cost;
                    RouteNode* neighbor = new RouteNode(new_x, new_y, target_layer, 
                                                        f_cost, g_cost, h_cost, current);
                    RouteNode* neighbor_old = allocated_map[neighbor_hash];
                    if (open_set.find(neighbor_hash) == open_set.end()) {
                        // if the position is not in the open list
                        if (!neighbor_old) {
                            delete neighbor_old;
                        }
                    } else {
                        // if the position is already in the open list, 
                        // we need to modify the g_cost of that entry by re-inserting
                        RouteNode* neighbor = new RouteNode(new_x, new_y, target_layer, 
                                                            f_cost, g_cost, h_cost, current);

                        RouteNode* neighbor_old = allocated_map[neighbor_hash];
                        open_list.erase(neighbor_old);
                        delete neighbor_old;
                    }
                    allocated_map[neighbor_hash] = neighbor;
                    open_list.insert(neighbor);
                    open_set.insert(neighbor_hash);
                }
            }
        }
        // free all allocated node
        for (auto p: allocated_map) {
            delete p.second;
        }
    }
    auto end_ = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_);
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_ - total_start);
    remaining_time_ms -= total_duration.count();
    if (duration.count() > 0) {  // Prevent division by zero
        double cur_num_nodes_per_ms = traversed_nodes / duration.count();
        num_nodes_per_ms = 0.3 * cur_num_nodes_per_ms + (1 - 0.3) * num_nodes_per_ms;
        printf("\n\nRemaining time: %fms\n", remaining_time_ms);
        printf("Max time per bump: %f, Max allowed nodes: %llu\n", max_run_time_per_bump, max_nodes_allowed);
        printf("h_scale: %f, Traversed: %llu, Duration: %lld ms, Nodes per ms: %f\n\n", h_scale, traversed_nodes, duration.count(), num_nodes_per_ms);
    }
    return path;
}

inline int Router::grid_hash(int x, int y, int layer) {
    // x: [1, 1500],
    // y: [1, 1000],
    // layer: [0, 1]
    return x + (y << 11) + (layer << 22);
}
inline int Router::get_opposite_direction(int dir) {
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
inline int Router::get_direction(int from_x, int from_y, int to_x, int to_y) {
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

std::vector<RouteEdge> Router::reconstruct_path(RouteNode* node, int start_x, int start_y) {
    // if it's in M2, add via manually
    std::vector<RouteEdge> path;
    if (node->layer == 1) {
        RouteEdge edge;
        edge.start_x = node->x;
        edge.start_y = node->y;
        edge.end_x = node->x;
        edge.end_y = node->y;
        edge.layer = 0;
        edge.is_via = true;
        path.push_back(edge);
    }
    RouteNode* trace = node;
    int end_x = trace->x, end_y = trace->y;
    while (trace->parent) {
        RouteEdge edge;
        edge.start_x = trace->parent->x;
        edge.start_y = trace->parent->y;
        edge.end_x = trace->x;
        edge.end_y = trace->y;
        edge.layer = trace->layer;
        edge.is_via = (trace->layer != trace->parent->layer);
        int dir = get_direction(edge.start_x, edge.start_y, trace->x, trace->y);
        layer_net_count[dir][trace->y][trace->x]++;
        layer_net_count[get_opposite_direction(dir)][trace->parent->y][trace->parent->x]++;
        // Merge the edges by only pushing the edge when met a turn.
        if (edge.is_via || (edge.start_x == start_x && edge.start_y == start_y)) {
            edge.end_x = end_x;
            edge.end_y = end_y;
            path.push_back(edge);
            end_x = edge.start_x;
            end_y = edge.start_y;
        }
        trace = trace->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}
inline bool Router::is_valid_move(int x, int y, int layer) {
    // Check grid bounds
    return (x < routing_area_gwidth && x >= 0 && y < routing_area_gheight && y >= 0);
}

inline int Router::calculate_heuristic(int x1, int y1, int x2, int y2) {
    // Manhattan distance
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}
inline double Router::calculate_g_cost(int from_x, int from_y, int from_layer, int to_x, int to_y, int to_layer) {
    int dir = get_direction(from_x, from_y, to_x, to_y);
    double wire_len = 0.0, move_cost = 0.0, net_via_cost = 0.0, ov_cost = 0.0;

    wire_len = (dir == LEFT || dir == RIGHT) ? grid_width : grid_height;
    move_cost = (from_layer != to_layer) ? (layer_costs[from_layer][from_y][from_x] + layer_costs[to_layer][from_y][from_x]) / 2 
                                                            :layer_costs[from_layer][from_y][from_x];
    net_via_cost = (from_layer != to_layer) * via_cost;
    ov_cost = (layer_net_count[dir][to_y][to_x] > edge_capacities[dir][to_y][to_x]) ? gcell_cost_max : 0;

    double g_cost = alpha * wire_len + beta * ov_cost +  _gamma * move_cost + delta * net_via_cost;
    return g_cost;
}
void Router::output_routing_results(std::ofstream& output_file) {
    int i = 0;
    for (const auto& net : routed_nets) {
        output_file << "n" << ++i << '\n';
        for (int i = 0 ; i < net.size() ; i++) {
            // Print net details
            if (net[i].is_via) {
                output_file << "via" << std::endl;
            }
            
            // Print wire information
            if (!(net[i].start_x == net[i].end_x && net[i].start_y == net[i].end_y)) {
                output_file << "M" << (net[i].layer + 1) << " " 
                            << routing_area_x + net[i].start_x * grid_width << " " << routing_area_y + net[i].start_y * grid_height << " "
                            << routing_area_x + net[i].end_x   * grid_width << " " << routing_area_y + net[i].end_y   * grid_height << std::endl;
            }
        }
        output_file << ".end" << std::endl;
    }
}
double Router::calculate_cost(const std::vector<std::vector<RouteEdge>>& routed_nets) {
    double wl = 0.0, ov = 0.0, gc = 0.0, via = 0.0;
    std::vector<double> real_costs;
    for (const auto& path: routed_nets) {
        double wlc = calculate_wire_length_cost(path);
        double gcc = calculate_gcell_cost(path);
        double viac = calculate_via_cost(path);
        wl += wlc;
        gc += gcc;
        via += viac;
        // std::cout << "Wire Length Cost (wl): " << wlc << "\n";
        // std::cout << "GCell Cost (gc): " << gcc << "\n";
        // std::cout << "Via Cost (via): " << viac << "\n";
        real_costs.push_back(wlc+gcc+viac);
    }
    ov = calculate_overflow_cost(); 
    double final_cost = alpha * wl + beta * ov + _gamma * gc + delta * via;
    // Print all values
    std::cout << "Wire Length Cost (wl): " << wl << "\n";
    std::cout << "Overflow Cost (ov): " << ov << "\n";
    std::cout << "GCell Cost (gc): " << gc << "\n";
    std::cout << "Via Cost (via): " << via << "\n";
    // std::cout << "Final Cost: " << final_cost << "\n";
    // printf("real net cost: %f, list: ", final_cost);
    // for (double x : real_costs) {
        // printf("%f ", x);
    // }
    // printf("\n");
    return final_cost;
}
double Router::calculate_wire_length_cost(const std::vector<RouteEdge>& path) {
    double total_length = 0.0;
    for (const auto& edge : path) {
        total_length += 
            std::abs((long) edge.end_x - (long) edge.start_x) * grid_width + 
            std::abs((long) edge.end_y - (long) edge.start_y) * grid_height;
    }
    return total_length;
}

double Router::calculate_overflow_cost() {
    // Calculate overflow
    double total_overflow = 0.0;
    // for (const auto& edge : path) {
        // int dir = get_direction(edge.start_x, edge.start_y, edge.end_x, edge.end_y);
    for (int i = 0 ; i < routing_area_gheight ; i++) {
        for (int j = 0 ; j < routing_area_gwidth ; j++) {
            for (int dir = 0 ; dir < 4 ; dir++) {
                if (layer_net_count[dir][i][j] > edge_capacities[dir][i][j]) {
                    total_overflow += 0.5 * std::max((long) layer_net_count[dir][i][j] - (long) edge_capacities[dir][i][j], 0L) * gcell_cost_max;
                }
            }
        }
    }
    return total_overflow / 2;
}


double Router::calculate_gcell_cost(const std::vector<RouteEdge>& path) {
    double total_cost = 0.0;
    for (const auto& edge: path) {
        // printf("(%lu, %lu)->(%lu, %lu)\n", edge.start_x, edge.start_y, edge.end_x, edge.end_y);
        if (edge.start_x == edge.end_x && edge.start_y == edge.end_y) continue;
        if (edge.is_via) {
            total_cost += 
                (layer_costs[0][edge.start_y][edge.start_x] + layer_costs[1][edge.start_y][edge.start_x]) / 2;
        } else {
            total_cost += layer_costs[edge.layer][edge.start_y][edge.start_x];
        }
    }
    auto last_edge = path.back();
    if (last_edge.start_x == last_edge.end_x && last_edge.start_y == last_edge.end_y) {
        total_cost += 
                (layer_costs[0][last_edge.end_y][last_edge.end_x] + layer_costs[1][last_edge.end_y][last_edge.end_x]) / 2;
    } else {
        total_cost += layer_costs[last_edge.layer][last_edge.end_y][last_edge.end_x];
    }
    return total_cost;
}
double Router::calculate_via_cost(const std::vector<RouteEdge>& path) {
    // // Count number of vias
    return via_cost * std::count_if(path.begin(), path.end(), 
        [](const RouteEdge& edge) { return edge.is_via; });
}
