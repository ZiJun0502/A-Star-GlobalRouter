#include "globals.h"
#include "chip.h"
#include "gcell.h"
#include "input_reader.h"

void read_grid_map(std::ifstream &file) {
    std::string line;
    int num_chips = 0;
    while (std::getline(file, line)) {
        // std::cout << line << '\n';
        if (line == ".ra") {
            std::getline(file, line);
            std::stringstream ss(line);
            ss >> routing_area_x >> routing_area_y >> routing_area_width >> routing_area_height;
        } else if (line == ".g") {
            std::getline(file, line);
            std::stringstream ss(line);
            ss >> grid_width >> grid_height;
        } else if (line == ".c") {
            std::getline(file, line);
            std::stringstream ss(line);
            double x, y, w, h;
            ss >> x >> y >> w >> h;
            Chip chip(x, y, w, h);
            // .b line
            std::getline(file, line);
            int idx;
            while(std::getline(file, line)) {
                if (line.empty()) break;
                std::stringstream ss(line);
                ss >> idx >> x >> y;
                chip.addBump(x, y);
            }
            chips[num_chips++] = chip;
        }
    }
    routing_area_gheight = routing_area_height / grid_height;
    routing_area_gwidth  = routing_area_width  / grid_width;
}

void read_edge_capacity(std::ifstream &file) {
    std::string line;
    int cnt = 0;
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        size_t left, bottom;
        ss >> left >> bottom;
        cnt++;
        edge_capacities.emplace_back(left, bottom);
    }
}

void read_cost_file(std::ifstream &file) {
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string type;
        ss >> type;
        if (type == ".alpha") {
            ss >> alpha;
        } else if (type == ".beta") {
            ss >> beta;
        } else if (type == ".gamma") {
            ss >> _gamma;
        } else if (type == ".delta") {
            ss >> delta;
        } else if (type == ".v") {
            ss >> via_cost;
        } else if (type == ".l") {
            std::vector<std::vector<double>> layer_costs(routing_area_gheight, std::vector<double>(routing_area_gwidth));
            double cost;
            for (size_t i = 0 ; i < routing_area_gheight ; i++) {
                std::getline(file, line);
                std::stringstream ss(line);
                for (size_t j = 0 ; j < routing_area_gwidth ; j++) {
                    ss >> cost;
                    layer_costs[i][j] = cost;
                }
            }
            if (layer1_costs.empty()) {
                layer1_costs = std::move(layer_costs);
            } else {
                layer2_costs = std::move(layer_costs);
            }
        }
    }
    layer1_gcells = std::vector<std::vector<GCell>>(routing_area_gheight, std::vector<GCell>(routing_area_gwidth));
    layer2_gcells = std::vector<std::vector<GCell>>(routing_area_gheight, std::vector<GCell>(routing_area_gwidth));
    
    for (size_t i = 0 ; i < routing_area_gheight ; i++) {
        for (size_t j = 0 ; j < routing_area_gwidth ; j++) {
            size_t idx = i*routing_area_gwidth + j;
            size_t left_cap  = edge_capacities[idx].first;
            size_t right_cap = edge_capacities[idx].second;
            layer1_gcells[i][j] = GCell(left_cap, right_cap, layer1_costs[i][j]);
        }
    }
    for (size_t i = 0 ; i < routing_area_gheight ; i++) {
        for (size_t j = 0 ; j < routing_area_gwidth ; j++) {
            size_t idx = i*routing_area_gwidth + j;
            size_t left_cap  = edge_capacities[idx].first;
            size_t right_cap = edge_capacities[idx].second;
            layer2_gcells[i][j] = GCell(left_cap, right_cap, layer2_costs[i][j]);
        }
    }
}

void read_input(int argc, char *argv[]) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <file.gmp> <file.gcl> <file.cst> <file.lg>" << std::endl;
        return;
    }
    std::cout << "Input Files:" << std::endl;
    std::cout << "File 1 (GMP): " << argv[1] << std::endl;
    std::cout << "File 2 (GCL): " << argv[2] << std::endl;
    std::cout << "File 3 (CST): " << argv[3] << std::endl;
    std::cout << "File 4 (LG): " << argv[4] << std::endl;
    std::ifstream grid_map_file(argv[1]);
    std::ifstream gcell_file(argv[2]);
    std::ifstream cost_file(argv[3]);
    output_filename = argv[4];

    if (!grid_map_file.is_open() || !gcell_file.is_open() || !cost_file.is_open()) {
        std::cerr << "Error opening input files." << std::endl;
        exit(EXIT_FAILURE);
    }

    read_grid_map(grid_map_file);
    read_edge_capacity(gcell_file);
    read_cost_file(cost_file);
    // build gcells grid

    grid_map_file.close();
    gcell_file.close();
    cost_file.close();
}