#ifndef PLANAR_H
#define PLANAR_H

#include <vector>
#include <unordered_set>

struct Edge{
    double weight;				// Edge weight
    double src_angle;   		// Angle (leaving) at source (relative to positive-x-axis)
    double dst_angle;   		// Angle (entering) at destination (relative to positive-x-axis)
	unsigned int src;			// Source id
    unsigned int dst;			// Destination id
    
	// Constructor for emplace_back
    Edge(unsigned int src, unsigned int dst, double weight, double src_angle, double dst_angle) : src(src), dst(dst), weight(weight), src_angle(src_angle), dst_angle(dst_angle) {}
};

struct Node{
	double radius;				// Radius for obstacles
	bool obstacle;				// Obstacle check
    unsigned int identity;		// Id starting from 1...n
    int x, y;					// Coordinate

    // Constructor for emplace_back
    Node(bool obstacle, unsigned int identity, int x, int y) : obstacle(obstacle), identity(identity), x(x), y(y) {}
};

struct PlanarGraph{
    std::vector<Node> nodes;
    std::vector<std::vector<Edge>> adj_list;
	unsigned int size;
};


/*
* Creates graph for user. User must provide function to check obstacles
* min_dist -> minimum length to connect vertex (meters)
* max_dist -> maximum length to connect vertex (meters)
* length -> length of operation area (meters)
* width -> width of operation area (meters)
* density -> desired density of the graph (vertecies per unit area [square meters])
*/
PlanarGraph * create_graph(double min_dist, double max_dist, double length, double width, double density);

int calculate_vertices(double length, double width, double density);
double calculate_angle(double x1, double y1, double x2, double y2);
double get_distance(int x1, int x2, int y1, int y2, double scale);
bool edge_intersected(int x1, int y1, int x2, int y2);
double calculate_angle(double x1, double y1, double x2, double y2);
void print_vertices(PlanarGraph *g);
void print_graph(PlanarGraph *g);


void create_obstacles(std::vector<Node> &nodes, std::unordered_set<unsigned int> &obstacles, const unsigned int size);

#endif