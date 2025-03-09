#include "planar.h"
#include <cmath>
#include <format>
#include <utility>
#include <cstddef>
#include <iostream>
#include <cmath>
#include <tuple>
#include <ranges>

// Custom hash function for std::pair<unsigned int, unsigned int>
struct PairHash {
	unsigned int operator()(const std::pair<unsigned int, unsigned int>& p) const {
		return std::hash<unsigned int>()(p.first) ^ (std::hash<unsigned int>()(p.second) << 1);
	}
};

Point absolute_coordinate(int x, int y, double resolution){
	return {static_cast<double>(x) / resolution, static_cast<double>(y) / resolution};
}


// TODO: Proximity Checks & Vehicle size buffer

/* TODO: Proximity and Vehicle Buffer
* User will pass in the WIDTH of the vehicle
* This will be used in buffer calculations
*/


/// @brief Creates a 2D geometric graph per the user specifications. Connects based on the specified length scale
/// @param min_dist Minimum connection distance 
/// @param max_dist Maximum connection distance
/// @param length Length of the graph in desired units
/// @param width Width of the graph in desired units
/// @param resolution The "cells per unit" of the graph. Positive, number, CAN be zero if there is no width constraint
/// @param min_proximity How close something can get to an obstacle
/// @return A constructed graph object
PlanarGraph * create_graph(const double min_dist, const double max_dist, const double length, const double width, const double resolution, const double min_proximity){
	
	// Parameter checking
	if(min_dist < 0.0 || max_dist < min_dist || length <= 0.0 || width <= 0.0 || resolution <= 0.0 || min_proximity < 0.0) return nullptr;
	
	PlanarGraph *g = new PlanarGraph;

	// Cell generation
	double area = length * width;

	// Ensure at least one vertex, calculate number of grid cells on l and w 
	unsigned int cell_rows = std::max(1.0, std::round(length * resolution));	// Ceil?
	unsigned int cell_cols = std::max(1.0, std::round(width * resolution));		// Ceil?

	// Set graph parameters
	g->cell_dimensions = {cell_cols, cell_rows};	// Store how many cells exist on each axis {x,y}
	g->vertex_count = cell_rows * cell_cols;
	g->resolution = resolution;
	g->geom_length = length;
	g->geom_width = width;

	// Allocate space in vectors
	g->vertices.reserve(g->vertex_count);
	g->adj_list.resize(g->vertex_count);
	// TODO: Calculate with dimensions

	// Insert vertices based on grid cells
	unsigned int id = 1;
	for(int x = 0; x < cell_cols; x++){
		for(int y = 0; y < cell_rows; y++){
			g->vertices.emplace_back(id, x, y);
			id++;
		}
	}	

	// Quick existing-edge lookup
	std::unordered_set<std::pair<unsigned int, unsigned int>, PairHash> edge_lookup;

    // Connect vertices
	for(auto &v1 : g->vertices){
		for(auto &v2 : g->vertices){
			if(v1.identity >= v2.identity) continue; // Eliminates self loops and duplicates

			// Confirm edge does not exist before anything else
			if(edge_lookup.find({v1.identity, v2.identity}) != edge_lookup.end()) continue;

			double dist = get_distance(v1.x, v1.y, v2.x, v2.y, resolution);

			// Check distances in O(1) first -- saves time
			if(dist < min_dist || dist > max_dist) continue;

			// TODO: Obstacles
			if(edge_intersected(v1.x, v2.x, v1.y, v2.y)){
				continue;
			}

			// Finally, check whether edge intersects an obstacle
			// Convert grid coordinate to user-units -- NAIVE APPROACH (no .5 center-cell approximations)
			Point absolute_p1 = absolute_coordinate(v1.x, v1.y, resolution);
			Point absolute_p2 = absolute_coordinate(v2.x, v2.y, resolution);

			// Check if it failed
			// ObstacleAnalysis ob = check_obstacle(absolute_p1, absolute_p2);
			// if(!ob.valid) {
			// 	continue;
			// }

			// Confirm not too close
			// if(ob.proximity < min_proximity){
			// 	continue;
			// }
			
			// After this point all edge checks have been passed, and the edge is valid 

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

/// @brief Calculates the Euclidian distance between two points given the desired resolution
/// @param x1 X coordinate of first vertex
/// @param y1 Y coordinate of first vertex
/// @param x2 X coordinate of second vertex
/// @param y2 Y coordinate of second vertex
/// @param resolution Resolution parameter given by user
/// @return The distance between both points
double get_distance(int x1, int y1, int x2, int y2, double resolution){
	double cell_size = 1.0 / resolution;
	
	// Calculate centered grid cell coordinates for accurate distance calculations
	double x1_ = (x1 + 0.5) * cell_size;
	double y1_ = (y1 + 0.5) * cell_size;
	double x2_ = (x2 + 0.5) * cell_size;
	double y2_ = (y2 + 0.5) * cell_size;

	double dx = x2_ - x1_;
	double dy = y2_ - y1_;
	
	return std::sqrt(dx * dx + dy * dy);
}

// Check whether an edge passes through a vertex
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

// Calculates edge angle between two vertices
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
	for(Node &n : g->vertices){
		std::cout << std::format("Node {} -- ({},{})\n", n.identity, n.x, n.y);
	}
}

void print_graph_information(PlanarGraph *g){
	std::cout << std::format("GRAPH -- | VERTICES: {:>5} | ROWS: {:>4} | COLS: {:>4} | LENGTH: {:>5}m | WIDTH: {:>5}m | AREA: {:>5}m | RESOLUTION: {} cell/m.\n\n",
		g->vertex_count, g->cell_dimensions.first, g->cell_dimensions.second, g->geom_length, g->geom_width, (g->geom_length * g->geom_width), g->resolution);
}

void print_graph(PlanarGraph *g){
	print_graph_information(g);


	for(auto &i : g->adj_list){
		for(auto &j : i){
			std::cout << std::format( "({}, {})  to  ({}, {})  DISTANCE: {:>8.2f}m  SRC_ANGLE: {:>8.2f}  DST_ANGLE: {:>8.2f}\n",
				g->vertices[j.src - 1].x, g->vertices[j.src - 1].y,
				g->vertices[j.dst - 1].x, g->vertices[j.dst - 1].y,
				j.weight, j.src_angle, j.dst_angle);
		}
	}
}

int main(){	
	// PlanarGraph *g = create_graph(1.0, std::numeric_limits<double>::max(), 10, 10, 1, 0.0);

	// if (g == nullptr || g->adj_list.empty()) {
	// 	std::cout << "Error: Graph creation failed or graph is empty\n";
	// 	return 1;
	// }

	// print_vertices(g);
	// print_graph(g);

	// delete g;

	PlanarGraph *a = create_graph(1.0, std::numeric_limits<double>::max(), 5, 5, 1, .5);
	PlanarGraph *b = create_graph(1.0, std::numeric_limits<double>::max(), 5, 5, 25.2, .5);
	// PlanarGraph *c = create_graph(1.0, std::numeric_limits<double>::max(), 30, 20, .5);
	// // PlanarGraph *d = create_graph(1.0, 20.0, 500, 500, 2.5);
	// PlanarGraph *e = create_graph(1.0, std::numeric_limits<double>::max(), 50, 10, 5);
	// PlanarGraph *f = create_graph(1.0, std::numeric_limits<double>::max(), 50, 10, 1);
	// // PlanarGraph *g = create_graph(1.0, std::numeric_limits<double>::max(), 32, 32, 2);
	// PlanarGraph *h = create_graph(1.0, std::numeric_limits<double>::max(), 10, 10, .25);

	print_graph(a);
	std::cout << "\n============================SECOND GRAPH============================\n\n";
	print_graph(b);
	// print_graph_information(c);
	// // print_graph_information(d);
	// print_graph_information(e);
	// print_graph_information(f);
	// // print_graph_information(g);
	// print_graph_information(h);

	delete a;
	delete b;
	// delete c;
	// // delete d;
	// delete e;
	// delete f;
	// // delete g;
	// delete h;


	// std::cout << "DISANCE TESTING" << std::endl;
	// for(int i : std::views::iota(0,20)){
	// 	distance_tests();
	// }
	
	// std::cout << std::format("Distance between (0,0) and (2,3) with 1 grid cells / meter: {:.2f}m\n", get_distance(0,0,2,3,1.0));
	// std::cout << std::format("Distance between (5,5) and (8,9) with 1 grid cell / meter: {:.2f}m\n", get_distance(5, 5, 8, 9, 1.0));
	// std::cout << std::format("Distance between (10,0) and (15,0) with 1 grid cell / meter: {:.2f}m\n", get_distance(10, 0, 15, 0, 1.0));
	// std::cout << std::format("Distance between (0,10) and (0,15) with 1 grid cell / meter: {:.2f}m\n", get_distance(0, 10, 0, 15, 1.0));
	// std::cout << std::format("Distance between (20,20) and (18,17) with 1 grid cell / meter: {:.2f}m\n", get_distance(20, 20, 18, 17, 1.0));
	// std::cout << std::format("Distance between (3,7) and (10,1) with 1 grid cell / meter: {:.2f}m\n", get_distance(3, 7, 10, 1, 1.0));
	
	// std::cout << std::format("Distance between (5,5) and (8,9) with 1 grid cell / meter: {:.2f}m\n\n", get_distance(5, 5, 8, 9, 1.0));
	// std::cout << std::format("Distance between (5,5) and (8,9) with 0.5 grid cell / meter: {:.2f}m\n\n", get_distance(5, 5, 8, 9, 0.5));
	// std::cout << std::format("Distance between (5,5) and (8,9) with 5 grid cell / meter: {:.2f}m\n\n", get_distance(5, 5, 8, 9, 5.0));
	// std::cout << std::format("Distance between (5,5) and (8,9) with 10 grid cell / meter: {:.2f}m\n\n", get_distance(5, 5, 8, 9, 10.0));
	// std::cout << std::format("Distance between (5,5) and (8,9) with 2 grid cell / meter: {:.2f}m\n\n", get_distance(5, 5, 8, 9, 2.0));
	// std::cout << std::format("Distance between (5,5) and (8,9) with 25 grid cell / meter: {:.2f}m\n\n", get_distance(5, 5, 8, 9, 25.0));
	// std::cout << std::format("Distance between (5,5) and (8,9) with 0.1 grid cell / meter: {:.2f}m\n\n", get_distance(5, 5, 8, 9, 0.1));
	// std::cout << std::format("Distance between (5,5) and (8,9) with 100 grid cell / meter: {:.2f}m\n\n", get_distance(5, 5, 8, 9, 100.0));
	// std::cout << std::format("Distance between (5,5) and (8,9) with 45 grid cell / meter: {:.2f}m\n\n", get_distance(5, 5, 8, 9, 45.0));


	/*
	* Case 1: 5x5 grid
	* Test with 10, 20, 40% coverage
	*/ 

	// double test_len_5 = 5.0;
	// double test_width_5 = 5.0;
	// double test_density_5 = 1;

	// unsigned int ideal_vertices = (test_len_5 * test_width_5) / test_density_5;
	// create_obstacles(ideal_vertices, .1);


	// Point conversion checking 
	// Point p1 = absolute_coordinate(0, 3, 1);
	// Point p2 = absolute_coordinate(10,10,2);
	// Point p3 = absolute_coordinate(100,100,5);
	// Point p4 = absolute_coordinate(35,23, 3);
	// Point p5 = absolute_coordinate(100, 3, 13);

	// std::cout << std::format("================\nPoint 1:\n\tCell/Unit: 1\n\tGrid Coordinates: (0,3)\n\tAbsolute Coordinates: ({},{})\n================\n", p1.x, p1.y);
	// std::cout << std::format("================\nPoint 2:\n\tCell/Unit: 2\n\tGrid Coordinates: (10,10)\n\tAbsolute Coordinates: ({},{})\n================\n", p2.x, p2.y);
	// std::cout << std::format("================\nPoint 3:\n\tCell/Unit: 5\n\tGrid Coordinates: (100,100)\n\tAbsolute Coordinates: ({},{})\n================\n", p3.x, p3.y);
	// std::cout << std::format("================\nPoint 4:\n\tCell/Unit: 3\n\tGrid Coordinates: 35,23)\n\tAbsolute Coordinates: ({},{})\n================\n", p4.x, p4.y);
	// std::cout << std::format("================\nPoint 5:\n\tCell/Unit: 13\n\tGrid Coordinates: (100,3)\n\tAbsolute Coordinates: ({},{})\n================\n", p5.x, p5.y);
}
