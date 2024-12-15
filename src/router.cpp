#include "router.h"
#include <chrono>

Router::Router() {
    layer_net_count[0] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    layer_net_count[1] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    layer_net_count[2] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    layer_net_count[3] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    
    max_run_time_per_bump = 6000 / (chips[0].bumps.size() - 1);
    h_scale = 1.0;
    first_net = true;
}


bool Router::route_nets() {
    // Route from each bump of chip[0] to each bump of chip[1]
    for (int i = 1; i < chips[0].bumps.size(); ++i) {
        std::pair<int, int> start = {chips[0].x + chips[0].bumps[i].first, chips[0].y + chips[0].bumps[i].second};
        std::pair<int, int> end = {chips[1].x + chips[1].bumps[i].first, chips[1].y + chips[1].bumps[i].second};
        
        auto start_ = std::chrono::steady_clock::now();
        std::cout << "Finding path for bump " << i << '\n'; 
        std::vector<RouteEdge> net_path = find_path(start, end);
        if (!net_path.empty()) {
            routed_nets.push_back(net_path);
        } else {
            // Path finding failed
            return false;
        }
        auto end_ = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_);
        std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;
    }
    return true;
}

std::vector<RouteEdge> Router::find_path(const std::pair<int, int>& start, 
                                         const std::pair<int, int>& end) {
    // Convert start and end to grid coordinates
    int start_x = static_cast<int>(start.first / grid_width);
    int start_y = static_cast<int>(start.second / grid_height);
    int end_x = static_cast<int>(end.first / grid_width);
    int end_y = static_cast<int>(end.second / grid_height);
    // Initial starting node
    auto start_tup = std::make_tuple(start_x, start_y, 0);

    static const int dx[] = {1, -1, 0, 0};
    static const int dy[] = {0, 0, 1, -1};
    // around 500 nodes per ms for 1500*1000
    // unsigned long long traversed_count = 0;
    unsigned long long expected_traversed_count = 2 * routing_area_gheight * routing_area_gwidth;
    unsigned long long max_nodes_allowed = max_run_time_per_bump * 500;

    bool path_found = false;
    bool up_scaled = false, down_scaled = false;
    bool rescaled = false;
    double up = 2, down = 0.8;
    while (!path_found) {
        std::set<RouteNode*, CompareRouteNode> open_list;
        std::unordered_map<std::tuple<int, int, int>, RouteNode*> allocated_map;
        std::unordered_set<std::tuple<int, int, int>> open_set;
        std::unordered_set<std::tuple<int, int, int>> closed_set;
        std::unordered_map<std::tuple<int, int, int>, double> g_hash_map;
        g_hash_map[start_tup] = 0;
        RouteNode* start_node = new RouteNode(start_x, start_y, 0, 0, 0, 
            calculate_heuristic(start_x, start_y, end_x, end_y));
        allocated_map[start_tup] = start_node;

        open_list.insert(start_node);
        open_set.insert(std::make_tuple(start_x, start_y, 0));
        unsigned long long traversed_nodes = 0;
        
        bool added = 0;
        while (!open_list.empty()) {
            RouteNode* current = *open_list.begin();
            auto current_tup = std::make_tuple(current->x, current->y, current->layer);

            open_list.erase(current);
            open_set.erase(current_tup);
            if (open_set.size() != open_list.size()) {
                printf("current: (%d, %d, %d)\n", current->x, current->y, current->layer);
                printf("current_tup: (%d, %d, %d)\n", std::get<0>(current_tup), std::get<1>(current_tup), std::get<2>(current_tup));
                std::cout << open_list.size() << ' ' << open_set.size() << '\n';
                break;
            }
            traversed_nodes++;
            // printf("current: (%d, %d, %d)\n", current->x, current->y, current->layer);
            if (first_net && traversed_nodes > max_nodes_allowed) {
                up_scaled = true;
                if (down_scaled) {
                    // Decay the scaling factors to prevent excessive oscillation
                    down_scaled = false;
                    up = std::max(1.0 + (up - 1.0) * 0.9, 1.05);
                }
                printf("up: %f\n", up);
                rescaled = true;
                h_scale *= up;  // Increase h_scale to prioritize heuristic more
                printf("traversed node: %llu, h_scale: %f\n", traversed_nodes, h_scale);
                break;
            }
            // Check if reached goal
            // printf("cur: (%d, %d, %d), end: (%d, %d, 0)\n", current->x, current->y, current->layer, end_x, end_y);

            // if (start_x == 3 && start_y == 7) {
            //     printf("nei: (%d, %d, %d)\n", current->x, current->y, current->layer);
            //     printf("g, h = (%f, %f)\n", current->g_cost, current->h_cost);
            //     // printf("not in g: %d\n", g_hash_map.find(current_tup) == g_hash_map.end());
            //     // printf("not in open: %d\n", open_set.find(current_tup) == open_set.end());
            //     // getchar();
            // }
            if (current->x == end_x && current->y == end_y) {
                if (first_net && rescaled && traversed_nodes < max_nodes_allowed * 0.5) {
                    // Continue scaling down
                    down = std::min(1.0 - (1.0 - down) * 0.9, 0.95);
                    h_scale *= down;
                    printf("down: %f, h_scale: %f\n", down, h_scale);
                    down_scaled = true;
                    
                    break;  // or break; depending on your precise logic
                }
                // Reconstruct path
                std::vector<RouteEdge> path;
                RouteNode* trace = current;
                // if it's in M2, add via manually
                if (current->layer == 1) {
                    RouteEdge edge;
                    edge.start_x = current->x;
                    edge.start_y = current->y;
                    edge.end_x = current->x;
                    edge.end_y = current->y;
                    edge.layer = 0;
                    edge.is_via = true;
                    path.push_back(edge);
                }
                int end_x = current->x, end_y = current->y;
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
                    if (edge.is_via || (edge.start_x == start_x && edge.start_y == start_y)) {
                        edge.end_x = end_x;
                        edge.end_y = end_y;
                        path.push_back(edge);
                        end_x = edge.start_x;
                        end_y = edge.start_y;
                    }
                    trace = trace->parent;
                }
                recorded_total_cost += current->g_cost;
                recorded_costs.push_back(current->g_cost);
                std::reverse(path.begin(), path.end());
                printf("traversed node: %llu, h_scale: %f\n", traversed_nodes, h_scale);
                
                first_net = false;
                return path;
            }

            // Mark as visited
            closed_set.insert(std::make_tuple(current->x, current->y, current->layer));

            for (int i = 0; i < 4; ++i) {
                int new_x = current->x + dx[i];
                int new_y = current->y + dy[i];

                // Check if move is valid
                if (!is_valid_move(new_x, new_y, current->layer)) continue;
                bool target_layer = (dy[i] == 0);
                auto neighbor_tup = std::make_tuple(new_x, new_y, target_layer);
                
                // Check if already in closed set
                if (closed_set.find(neighbor_tup) != closed_set.end()) 
                    continue;

                // M1: vertical, M2: horizontal
                double g_cost = g_hash_map[current_tup] + calculate_g_cost(current->x, current->y, current->layer, new_x, new_y, target_layer);
                double h_cost = h_scale * calculate_heuristic(new_x, new_y, end_x, end_y);
                double f_cost = g_cost + h_cost;
                // printf("g, h: %f, %f\n", g_cost, h_cost);
                if (g_hash_map.find(neighbor_tup) == g_hash_map.end() || g_cost < g_hash_map[neighbor_tup]) {
                    // Create new node
                    g_hash_map[neighbor_tup] = g_cost;
                    if (open_set.find(neighbor_tup) == open_set.end()) {
                        RouteNode* neighbor = new RouteNode(new_x, new_y, target_layer, 
                                                            f_cost, g_cost, h_cost, current);

                        if (allocated_map.find(neighbor_tup) != allocated_map.end()) {
                            RouteNode* neighbor_old = allocated_map[neighbor_tup];
                            delete neighbor_old;
                        }
                        allocated_map[neighbor_tup] = neighbor;
                        open_list.insert(neighbor);
                        open_set.insert(neighbor_tup);
                    } else {
                        RouteNode* neighbor = new RouteNode(new_x, new_y, target_layer, 
                                                            f_cost, g_cost, h_cost, current);

                        RouteNode* neighbor_old = allocated_map[neighbor_tup];
                        open_list.erase(neighbor_old);
                        delete neighbor_old;
                        allocated_map[neighbor_tup] = neighbor;
                        open_list.insert(neighbor);
                        open_set.insert(neighbor_tup);
                    }
                }
            }
        }
    }
    // No path found
    return {};
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
inline bool Router::is_valid_move(int x, int y, int layer) {
    // Check grid bounds
    return (x < routing_area_gwidth && x >= 0 && y < routing_area_gheight && y >= 0);
}

inline double Router::calculate_heuristic(int x1, int y1, int x2, int y2) {
    // Manhattan distance
    return std::abs(static_cast<int>(x1) - static_cast<int>(x2)) + 
           std::abs(static_cast<int>(y1) - static_cast<int>(y2));
}
inline double Router::calculate_g_cost(int from_x, int from_y, int from_layer, int to_x, int to_y, int to_layer) {
    int dir = get_direction(from_x, from_y, to_x, to_y);
    double wire_len = 0.0, move_cost = 0.0, net_via_cost = 0.0, ov_cost = 0.0;
    wire_len = (dir == LEFT || dir == RIGHT) ? grid_width : grid_height;
    // printf("(%d, %d, %d), (%d, %d, %d)\n", from_layer, from_y, from_x, to_layer, to_y, to_x);
    // printf("(%d, %d), (%d, %d)", layer_costs[0].size(), layer_costs[0][0].size(), layer_costs[1].size(), layer_costs[1][0].size());
    move_cost = (from_layer != to_layer) ? (layer_costs[from_layer][from_y][from_x] + layer_costs[to_layer][from_y][from_x]) / 2 
                                                            :layer_costs[from_layer][from_y][from_x];
    net_via_cost = (from_layer != to_layer) * via_cost;
    // printf("dir: %d, to_y: %d, to_x: %d\n", dir, to_y, to_x);
    ov_cost = (layer_net_count[dir][to_y][to_x] > edge_capacities[dir][to_y][to_x]) ?
        gcell_cost_max : 0;
        // 0.5 * std::max((long) layer_net_count[dir][to_y][to_x] - (long) edge_capacities[dir][to_y][to_x], 0L) * gcell_cost_max +
        // 0.5 * std::max((long) layer_net_count[get_opposite_direction(dir)][to_y][to_x] - (long) edge_capacities[get_opposite_direction(dir)][to_y][to_x], 0L) * gcell_cost_max;
    double g_cost = alpha * wire_len + beta * ov_cost +  _gamma * move_cost + delta * net_via_cost;
    // printf("%f ", g_cost);
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
