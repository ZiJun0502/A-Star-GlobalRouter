#include "router.h"

// Pthread routine for routing a single bump
void* route_net_thread(void* arg) {
    ThreadArgs* args = static_cast<ThreadArgs*>(arg);
    // Create a local BidirectionalAStar instance
    BidirectionalAStar astar;
    astar.layer_net_count_rwlock = args->layer_net_count_rwlock;
    
    
    int start_net, end_net;
    while (args->pool->get_work(start_net, end_net)) {
        for (int i = start_net ; i < end_net ; i++) {
            auto start_time = std::chrono::steady_clock::now();
            std::pair<int, int> start = {
                chips[0].x + chips[0].bumps[i+1].first, 
                chips[0].y + chips[0].bumps[i+1].second
            };
            std::pair<int, int> end = {
                chips[1].x + chips[1].bumps[i+1].first, 
                chips[1].y + chips[1].bumps[i+1].second
            };
            printf("Thread id: %d, net: %d, (%d, %d), (%d, %d)\n", args->id, i, start.first, start.second, end.first, end.second);
            Router::routed_nets[i] = astar.find_path(start, end);
            
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            std::cout << "Time taken for net " << i << ": " 
                    << duration.count() << " ms" << std::endl;
        }
    }
    return nullptr;
}
Router::Router() {
    Router::layer_net_count[0] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    Router::layer_net_count[1] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    Router::layer_net_count[2] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    Router::layer_net_count[3] = std::vector<std::vector<int>>(routing_area_gheight, std::vector<int>(routing_area_gwidth, 0));
    
    max_run_time_per_bump = 400000 / (chips[0].bumps.size() - 1);
    num_nodes_per_ms = 500;
    h_scale = 1.0;
    up = 2.0, down = 0.8;
    rescaled = false;
    for (int i = 0 ; i < num_locks; i++) {
        if (pthread_rwlock_init(&layer_net_count_rwlock, nullptr) != 0) {
            std::cerr << "Failed to initialize read-write lock" << std::endl;
            throw std::runtime_error("Read-write lock initialization failed");
        }
    }
    routed_nets.resize((chips[0].bumps.size() - 1));
}
Router::~Router() {
    for (int i = 0 ; i < num_locks; i++) {
        pthread_rwlock_destroy(&layer_net_count_rwlock);
    }
}


std::vector<std::vector<RouteEdge>> Router::route_nets() {
    remaining_time_ms = 400000;
    const int num_threads = 8;
    // Prepare thread-related structures
    std::vector<pthread_t> threads(num_threads);
    std::vector<ThreadArgs> thread_args(num_threads);
    std::vector<std::vector<RouteEdge>> bump_paths(num_threads);
    // std::vector<bool> thread_success(num_threads, false);
    bool *thread_success = new bool[num_threads];
    memset(thread_success, 0, sizeof(bool) * (num_threads));
    
    ThreadPool pool((chips[0].bumps.size() - 1));
    // Create threads for each bump
    for (int i = 0; i < num_threads; ++i) {
        // Calculate max runtime for this bump
        // int max_run_time_per_bump = remaining_time_ms / (num_bumps - i);
        // Setup thread arguments
        thread_args[i] = ThreadArgs(
            i,
            this,               // Router instance
            &(thread_success[i]),  // Success flag
            max_run_time_per_bump,
            &pool,
            layer_net_count_rwlock  // Mutex for layer_net_count
        );
        // Create thread
        int rc = pthread_create(
            &threads[i], 
            nullptr, 
            route_net_thread, 
            &thread_args[i]
        );
        
        if (rc) {
            std::cerr << "Error creating thread for bump " << i << std::endl;
            return std::vector<std::vector<RouteEdge>>();
        }
    }
    for (int i = 0; i < num_threads; ++i) {
        pthread_join(threads[i], NULL);
    }
    return Router::routed_nets;
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
std::vector<std::vector<RouteEdge>> Router::routed_nets;