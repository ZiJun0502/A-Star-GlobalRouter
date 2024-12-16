#ifndef ROUTE_NODE_H
#define ROUTE_NODE_H

#include <iostream>
#include <vector>

class RouteNode {
public:
    int x;
    int y;
    int layer; // 0 for M1, 1 for M2
    double f_cost; // Total estimated cost
    double g_cost; // Cost from start
    double h_cost; // Heuristic cost to goal
    RouteNode* parent;

    RouteNode(int x, int y, int layer, double f = 0, double g = 0, double h = 0, RouteNode* p = nullptr)
        : x(x), y(y), layer(layer), f_cost(f), g_cost(g), h_cost(h), parent(p) {}

    // Comparison for priority queue
    bool operator>(const RouteNode& other) const {
        return f_cost > other.f_cost;
    }
    bool operator<(const RouteNode& other) const { return f_cost < other.f_cost; }
};
struct CompareRouteNode {
    bool operator()(const RouteNode* lhs, const RouteNode* rhs) const {
        if (lhs->f_cost != rhs->f_cost) 
            return lhs->f_cost < rhs->f_cost;
        if (lhs->x != rhs->x) 
            return lhs->x < rhs->x;
        if (lhs->y != rhs->y) 
            return lhs->y < rhs->y;
        return lhs->layer < rhs->layer;
    }
};
class RouteEdge {
public:
    int start_x, start_y, end_x, end_y;
    int layer;
    bool is_via;

    RouteEdge(int start_x = 0, int start_y = 0, int end_x = 0, int end_y = 0, int layer = 0, bool is_via = false)
        : start_x(start_x), start_y(start_y), end_x(end_x), end_y(end_y), layer(layer), is_via(is_via) {}
};


std::ostream& operator<<(std::ostream& os, const std::vector<RouteEdge>& path);
#endif