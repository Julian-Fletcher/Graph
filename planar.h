#include <vector>
#include <unordered_set>

struct Edge{
    unsigned int src;            // Source id
    unsigned int dst;            // Destination id
    double weight;      // Edge weight
    double src_angle;   // Angle (leaving) at source (relative to positive-x-axis)
    double dst_angle;   // Angle (entering) at destination (relative to positive-x-axis)

    // Constructor for emplace_back
    Edge(unsigned int src, unsigned int dst, double weight, double src_angle, double dst_angle) : src(src), dst(dst), weight(weight), src_angle(src_angle), dst_angle(dst_angle) {}
};

struct Node{
	bool obstacle;
    unsigned int identity;		// Id starting from 1...n
    int x, y;					// Coordinate
	double radius;				// Radius for obstacles

    // Constructor for emplace_back
    Node(bool obstacle, unsigned int identity, int x, int y) : obstacle(obstacle), identity(identity), x(x), y(y) {}
};

struct PlanarGraph{
    unsigned int size;
    double vertex_distance;
    std::vector<Node> nodes;
    std::vector<std::vector<Edge>> adj_list;
};


PlanarGraph *create_graph(unsigned int n, double min_dist, double max_dist);
void create_connections(PlanarGraph *g);
double calculate_angle(double x1, double y1, double x2, double y2);
double get_distance(int x1, int x2, int y1, int y2, float scale);
bool edge_intersected(int x1, int y1, int x2, int y2);
double calculate_angle(double x1, double y1, double x2, double y2);
void print_vertices(PlanarGraph *g);
void print_graph(PlanarGraph *g);
void create_obstacles(std::vector<Node> &nodes, std::unordered_set<unsigned int> &obstacles, const unsigned int size);