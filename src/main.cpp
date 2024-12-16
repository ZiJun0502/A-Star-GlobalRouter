#include "globals.h"
#include "input_reader.h"
#include "router.h"
#include <chrono>

int main(int argc, char *argv[]) {
    auto start = std::chrono::steady_clock::now();
    read_input(argc, argv);
    // print_globals();
    // printf("Hello\n");
    Router router;
    router.route_nets();
    std::ofstream output_file(output_filename);
    router.output_routing_results(output_file);

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count() << " us" << std::endl;

    return 0;
}
