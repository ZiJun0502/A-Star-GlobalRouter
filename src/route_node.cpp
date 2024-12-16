#include "route_node.h"

std::ostream& operator<<(std::ostream& os, const std::vector<RouteEdge>& path) {
    for (const auto& edge : path) {
        if (edge.is_via) {
            // Format: (layer, start_x, start_y, end_x, end_y) -> via -> (layer, start_x, start_y, ...)
            os << " -> via\n -> [M" << edge.layer << ", (" << edge.start_x << ", " << edge.start_y 
               << "), (" << edge.end_x << ", " << edge.end_y << ")]\n";
        } else {
            // Format: (layer, start_x, start_y, end_x, end_y) -> (layer, start_x, ...)
            os << " -> [M" << edge.layer << ", (" << edge.start_x << ", " << edge.start_y 
               << "), (" << edge.end_x << ", " << edge.end_y << ")]\n";
        }
    }
    return os;
}

