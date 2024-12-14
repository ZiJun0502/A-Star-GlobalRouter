#include "router.h"

Router::Router() {
    // Initialize net count tracking for each layer and grid cell
    layer_net_count.resize(2, 
        std::vector<std::vector<size_t>>(routing_area_gheight, 
            std::vector<size_t>(routing_area_gwidth, 0)));
}

bool Router::route_nets() {
    // Initialize grid costs
    initialize_grid_costs();

    // Route from each bump of chip[0] to each bump of chip[1]
    for (size_t i = 1; i < chips[0].bumps.size(); ++i) {
        std::pair<size_t, size_t> start = {chips[0].x + chips[0].bumps[i].first, chips[0].y + chips[0].bumps[i].second};
        std::pair<size_t, size_t> end = {chips[1].x + chips[1].bumps[i].first, chips[1].y + chips[1].bumps[i].second};
        
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

std::vector<RouteEdge> Router::find_path(const std::pair<size_t, size_t>& start, 
                                         const std::pair<size_t, size_t>& end) {
    // Convert start and end to grid coordinates
    size_t start_x = static_cast<size_t>(start.first / grid_width);
    size_t start_y = static_cast<size_t>(start.second / grid_height);
    size_t end_x = static_cast<size_t>(end.first / grid_width);
    size_t end_y = static_cast<size_t>(end.second / grid_height);
    // std::cout << start_x << ' ' << start_y << ' ' << end_x << ' ' << end_y << '\n';
    // Priority queue for A* search
    std::priority_queue<RouteNode*, std::vector<RouteNode*>, 
                        std::greater<RouteNode*>> open_list;
    std::unordered_set<std::tuple<size_t, size_t, size_t>> closed_set;
    std::unordered_map<std::tuple<size_t, size_t, size_t>, double> g_hash_map;
    g_hash_map[std::make_tuple(start_x, start_y, 0)] = 0;
    // Initial nodes for both layers
    RouteNode* start_node_m1 = new RouteNode(start_x, start_y, 0, 0, 0, 
        calculate_heuristic(start_x, start_y, end_x, end_y));
    // RouteNode* start_node_m2 = new RouteNode(start_x, start_y, 1, 0, 0, 
    //     calculate_heuristic(start_x, start_y, end_x, end_y));

    open_list.push(start_node_m1);
    // open_list.push(start_node_m2);
    // std::cout << "Ending coordinate: ";
    // std::cout << end_x << ' ' << end_y << '\n';
    const int dx[] = {1, -1, 0, 0};
    const int dy[] = {0, 0, 1, -1};
    while (!open_list.empty()) {
        RouteNode* current = open_list.top();
        open_list.pop();

        // Check if reached goal
        // getchar();
        // std::cout << current->x << ' ' << current->y << '\n';
        if (current->x == end_x && current->y == end_y) {
            // Reconstruct path
            std::vector<RouteEdge> path;
            RouteNode* trace = current;
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
            while (trace->parent) {
                RouteEdge edge;
                edge.start_x = trace->parent->x;
                edge.start_y = trace->parent->y;
                edge.end_x = trace->x;
                edge.end_y = trace->y;
                edge.layer = trace->layer;
                edge.is_via = (trace->layer != trace->parent->layer);
                path.push_back(edge);
                trace = trace->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Mark as visited
        closed_set.insert(std::make_tuple(current->x, current->y, current->layer));


        for (int i = 0; i < 4; ++i) {
            size_t new_x = current->x + dx[i];
            size_t new_y = current->y + dy[i];

            // Check if move is valid
            if (!is_valid_move(new_x, new_y, current->layer)) continue;
            auto neighbor_tup = std::make_tuple(new_x, new_y, current->layer);
            // Check if already in closed set
            if (closed_set.find(neighbor_tup) != closed_set.end()) 
                continue;

            // M1: vertical, M2: horizontal
            bool target_layer = (dy[i] == 0);

            // Calculate costs
            double move_cost = 0, via_cost = 0, g_cost = 0;
            if (target_layer) {
                // M2
                move_cost = layer2_costs[new_y][new_x];
            } else {
                // M1
                move_cost = layer1_costs[new_y][new_x];
            }
            // Add via
            if (current->layer != target_layer) {
                via_cost = (layer1_costs[current->y][current->x] + layer2_costs[current->y][current->x]) / 2;
            }
            g_cost = current->g_cost + delta * via_cost + _gamma * move_cost;
            double h_cost = calculate_heuristic(new_x, new_y, end_x, end_y);
            double f_cost = g_cost + h_cost;
            if (g_hash_map.find(neighbor_tup) == g_hash_map.end() || g_cost < g_hash_map[neighbor_tup]) {
                // printf("new_x: %lu, new_y: %lu, g_cost: %f, h_cost: %f, f_cost: %f\n", new_x, new_y, g_cost, h_cost, f_cost);
                // Create new node
                g_hash_map[neighbor_tup] = g_cost;
                RouteNode* neighbor = new RouteNode(new_x, new_y, target_layer, 
                                                    f_cost, g_cost, h_cost, current);
                open_list.push(neighbor);
            }
        }
    }

    // No path found
    return {};
}

inline bool Router::is_valid_move(size_t x, size_t y, size_t layer) {
    // Check grid bounds
    return !(x >= routing_area_gwidth || y >= routing_area_gheight);
}

double Router::calculate_heuristic(size_t x1, size_t y1, size_t x2, size_t y2) {
    // Manhattan distance
    return std::abs(static_cast<int>(x1) - static_cast<int>(x2)) + 
           std::abs(static_cast<int>(y1) - static_cast<int>(y2));
}

void Router::initialize_grid_costs() {
    // Reset net counts
    for (auto& layer : layer_net_count) {
        for (auto& row : layer) {
            std::fill(row.begin(), row.end(), 0);
        }
    }
}

void Router::output_routing_results(std::ofstream& output_file) {
    int i = 0;
    for (const auto& net : routed_nets) {
        output_file << "n" << ++i << '\n';
        for (size_t i = 0 ; i < net.size() ; i++) {
            // Print net details
            if (net[i].is_via) {
                output_file << "via" << std::endl;
            }
            
            // Print wire information
            output_file << "M" << (net[i].layer + 1) << " " 
                        << routing_area_x + net[i].start_x * grid_width << " " << routing_area_y + net[i].start_y * grid_height << " "
                        << routing_area_x + net[i].end_x   * grid_width << " " << routing_area_y + net[i].end_y   * grid_height << std::endl;
        }
        output_file << ".end" << std::endl;
    }
}

double Router::calculate_wire_length(const std::vector<RouteEdge>& path) {
    double total_length = 0.0;
    for (const auto& edge : path) {
        total_length += std::sqrt(
            std::pow(edge.end_x - edge.start_x, 2) + 
            std::pow(edge.end_y - edge.start_y, 2)
        );
    }
    return total_length;
}

double Router::calculate_overflow(const std::vector<RouteEdge>& path) {
    // Reset net counts
    for (auto& layer : layer_net_count) {
        for (auto& row : layer) {
            std::fill(row.begin(), row.end(), 0);
        }
    }

    // Count nets on each edge
    double max_cell_cost = 0.0;
    for (const auto& edge : path) {
        // Update net count and track max cell cost
        layer_net_count[edge.layer][edge.end_y][edge.end_x]++;
        
        // Determine max cell cost based on initial grid configuration
        max_cell_cost = std::max(max_cell_cost, 
            edge.layer == 0 ? layer1_gcells[edge.end_y][edge.end_x].cost : 
                              layer2_gcells[edge.end_y][edge.end_x].cost);
    }

    // Calculate overflow
    double total_overflow = 0.0;
    for (size_t layer = 0; layer < 2; ++layer) {
        for (size_t y = 0; y < routing_area_gheight; ++y) {
            for (size_t x = 0; x < routing_area_gwidth; ++x) {
                // Determine edge capacity
                size_t edge_capacity = layer == 0 ? 
                    layer1_gcells[y][x].left_edge_capacity : 
                    layer2_gcells[y][x].left_edge_capacity;
                
                // Calculate overflow
                if (layer_net_count[layer][y][x] > edge_capacity) {
                    total_overflow += 
                        (layer_net_count[layer][y][x] - edge_capacity) * 0.5 * max_cell_cost;
                }
            }
        }
    }

    return total_overflow;
}

double Router::calculate_via_count(const std::vector<RouteEdge>& path) {
    // Count number of vias
    return std::count_if(path.begin(), path.end(), 
        [](const RouteEdge& edge) { return edge.is_via; });
}
