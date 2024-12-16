#include "router.h"

Router::Router() {
    Router::layer_net_count[0] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    Router::layer_net_count[1] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    Router::layer_net_count[2] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    Router::layer_net_count[3] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
}


bool Router::route_nets() {
    // Route from each bump of chip[0] to each bump of chip[1]
    BidirectionalAStar astar;
    double remaining_time_ms = 100000;
    double num_nodes_per_ms = 500;
    double avg_traversed_nodes = 4 * routing_area_gheight + routing_area_gwidth;
    double up = 2.0, down = 0.8;
    double max_runtime_per_bump = 400000 / (chips[0].bumps.size() - 1);
    bool rescaled = false;
    for (int i = 1; i < chips[0].bumps.size(); ++i) {
        std::pair<int, int> start = {chips[0].x + chips[0].bumps[i].first, chips[0].y + chips[0].bumps[i].second};
        std::pair<int, int> end = {chips[1].x + chips[1].bumps[i].first, chips[1].y + chips[1].bumps[i].second};
        
        std::cout << "Finding path for bump " << i << '\n'; 
        auto bump_start_time = std::chrono::steady_clock::now();
        std::vector<RouteEdge> net_path;
        while(true) {
            // set runtime constraint for A*;
            max_runtime_per_bump = remaining_time_ms / (chips[0].bumps.size() - i);
            astar.max_nodes_allowed = (unsigned long long) std::max(max_runtime_per_bump * num_nodes_per_ms, (double) 4 * routing_area_gheight + routing_area_gwidth);

            auto start_time = std::chrono::steady_clock::now();
            net_path = astar.find_path(start, end);
            if (i == 17) {
                std::cout << net_path;
            }
            auto end_time = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> duration = end_time - start_time;

            remaining_time_ms -= duration.count();
            avg_traversed_nodes = 0.3 * astar.traversed_nodes + (1 - 0.3) * avg_traversed_nodes;
            // unable to find path within limited time, upscale h
            if (net_path.empty()) {
                up = std::max(1.0 + (up - 1.0) * 0.9, 1.4);
                rescaled = true;
                astar.h_scale *= up;
                printf("\n\nRemaining time: %fms, duration: %f\n", remaining_time_ms, duration.count());
                printf("Max time per bump: %f, Max allowed nodes: %llu\n", max_runtime_per_bump, astar.max_nodes_allowed);
                printf("h_scale: %f, Traversed: %llu, Nodes per ms: %f\n\n", astar.h_scale, astar.traversed_nodes, num_nodes_per_ms);
            } else {
                if (rescaled && avg_traversed_nodes < astar.max_nodes_allowed * 0.1) {
                    // Continue scaling down
                    down = std::min(1.0 - (1.0 - down) * 0.9, 0.95);
                    astar.h_scale *= down;
                }
                double cur_num_nodes_per_ms = (double) astar.traversed_nodes / duration.count();
                num_nodes_per_ms = 0.3 * cur_num_nodes_per_ms + (1 - 0.3) * num_nodes_per_ms;
                printf("Remaining time: %fms, duration: %f\n", remaining_time_ms, duration.count());
                printf("h_scale: %f, Traversed: %llu, Nodes per ms: %f\n\n", astar.h_scale, astar.traversed_nodes, num_nodes_per_ms);
                break;
            }
        }
        auto bump_end_time = std::chrono::steady_clock::now();
        auto bump_duration = std::chrono::duration_cast<std::chrono::milliseconds>(bump_end_time - bump_start_time);
        // std::cout << "Time to route bump " << i << ": " << bump_duration.count() << " seconds\n";
        routed_nets.push_back(net_path);
    }
    return true;
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

std::array<std::vector<std::vector<int>>, 4> Router::layer_net_count;