#include "planar.h"
// #define _USE_MATH_DEFINES
#include <cmath>
#include <format>
#include <utility>
#include <cstddef>
#include <iostream>
#include <cmath>

// Custom hash function for std::pair<size_t,size_t>
struct PairHash {
    unsigned int operator()(const std::pair<unsigned int, unsigned int>& p) const {
        return std::hash<unsigned int>()(p.first) ^ (std::hash<unsigned int>()(p.second) << 1);
    }
};

// Determines number of vertices needed given the density of the graph
int calculate_vertices(double length, double width, double density){
	// Area of the graph space
	double area = length * width;

	// Ideal number of vertices based on density and area.  We use floor to get an integer.
	int vertices = static_cast<int>(std::max(1.0, ceil(density * area)));

	return vertices;
}

// Obstalce Generation
// Proximity Checking for vertices/obstacles --> Buffer around user vehicle
PlanarGraph * create_graph(double min_dist, double max_dist, double length, double width, double density){
    
	// Parameter checking
	if(min_dist < 0.0 || max_dist < min_dist || length <= 0.0 || width <= 0.0 || density <= 0.0) return nullptr;
	
	PlanarGraph *g = new PlanarGraph;

    g->size = calculate_vertices(length, width, density);

    // Allocate space in vectors
	g->nodes.reserve(g->size * g->size);
    g->adj_list.resize(g->size * g->size);

	// Quick existing-edge lookup
	std::unordered_set<std::pair<unsigned int, unsigned int>, PairHash> edge_lookup;

    // Add vertices
    unsigned int id = 1;
    for(size_t x = 0; x < g->size; x++){
        for(size_t y = 0; y < g->size; y++){
            g->nodes.emplace_back(false, id, x, y);
            id++;
        }
    }

	// OBSTACLE CHECKING NEEDS TO BE DONE HERE

    // Conect vertices
    for(auto &v1 : g->nodes){
        if(v1.obstacle) continue;	// Skip if obstacle
		for(auto &v2 : g->nodes){
            if(v2.obstacle) continue;	// Skip if obstacle
			if(v1.identity >= v2.identity) continue; // Eliminate self loops and duplicates
			

            double dist = get_distance(v1.x, v2.x, v1.y, v2.y, density);

			// Exit loop if not within valid connection range [min_dist, max_dist]
			if(dist < min_dist || dist > max_dist || edge_intersected(v1.x, v2.x, v1.y, v2.y)){
				continue;
			}
			
			if(edge_lookup.find({v1.identity, v2.identity}) != edge_lookup.end()) continue;

			double src_angle = calculate_angle(v1.x, v1.y, v2.x, v2.y);
			
			// Opposite of source angle
			double dst_angle = std::fmod(src_angle + 180.0, 360.0);

			// Undirected Graph, for directed remove 2nd insertion
			g->adj_list[v1.identity-1].emplace_back(v1.identity, v2.identity, dist, src_angle, dst_angle);
			g->adj_list[v2.identity-1].emplace_back(v2.identity, v1.identity, dist, dst_angle, src_angle);

			// Insert into set
			edge_lookup.insert({v1.identity, v2.identity});
			edge_lookup.insert({v2.identity, v1.identity});
        }
    }
    return g;
}

// Euclidian distance scaled by distance between each vertex
double get_distance(int x1, int x2, int y1, int y2, double density) {
    double area_per_vertex = 1.0 / density;
	double scale = std::sqrt(area_per_vertex);

	return std::sqrt(std::pow(static_cast<double>(x2-x1) * scale, 2) + std::pow(static_cast<double>(y2-y1) * scale, 2));
}

// Check whether an edge passes through a vertex or obstacle*
// Returns true if it does
bool edge_intersected(int x1, int y1, int x2, int y2) {
    // If the two points are the same, no edge exists
    if (x1 == x2 && y1 == y2) return true;

    // Get deltas
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);

    // Special cases for vertical/horizontal lines
    if (dx == 0) return dy > 1; // Vertical line
    if (dy == 0) return dx > 1; // Horizontal line

    // Use Bresenham's line algorithm to check for intermediate grid points
    int x = x1, y = y1;
    int x_inc = (x2 > x1) ? 1 : -1;
    int y_inc = (y2 > y1) ? 1 : -1;
    int error = dx - dy;

    while (true) {
        // If we reach the destination, no intersection
        if (x == x2 && y == y2) break;

        // Check if the current point is a grid point other than the start and end
        if ((x != x1 || y != y1) && (x != x2 || y != y2)) {
            return true; // Intersection detected
        }

        int error2 = 2 * error;
        if (error2 > -dy) {
            error -= dy;
            x += x_inc;
        }
        if (error2 < dx) {
            error += dx;
            y += y_inc;
        }
    }

    return false; // No intersection
}

double calculate_angle(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    double angle = std::atan2(dy, dx);
	angle *= (180.0 / M_PI);

    // Normalize to [0, 360)
	if (angle < 0) {
        angle += 360;
    }
	return angle;
}


void print_vertices(PlanarGraph *g){
	for(Node &n : g->nodes){
		std::cout << std::format("Node {} -- ({},{}) Obstacle: {}\n", n.identity, n.x, n.y, n.obstacle);
	}
}

void print_graph(PlanarGraph *g){
    for(auto &i : g->adj_list){
        for(auto &j : i){
            std::cout << std::format( "({}, {})  to  ({}, {})  DISTANCE: {:>8.2f}  SRC_ANGLE: {:>8.2f}  DST_ANGLE: {:>8.2f}\n",
                g->nodes[j.src - 1].x, g->nodes[j.src - 1].y,
                g->nodes[j.dst - 1].x, g->nodes[j.dst - 1].y,
                j.weight, j.src_angle, j.dst_angle);
        }
    }
}

int main(){
	
    PlanarGraph *g = create_graph(1.0, std::numeric_limits<double>::max(), 10, 10, .1);

    if (g == nullptr || g->adj_list.empty()) {
        std::cout << "Error: Graph creation failed or graph is empty\n";
        return 1;
    }

	// print_vertices(g);
    print_graph(g);

    delete g;
}