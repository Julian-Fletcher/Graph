#include "planar.h"
#include <random>

std::unordered_set<std::pair<unsigned int, unsigned int>> obstacles;


// Randomly select vertices to become obstacles
// Coverage is % of graph that should be covered by obstacles 

// Need to know --- 
	// # Vertices
	// Coordinates of vertices
	// Or expect the algorithm to match what is expected <---
void create_obstacles(unsigned int ideal_vertices, double coverage){
	// Create generator
	std::mt19937 engine (std::random_device{}());	
	std::uniform_int_distribution<unsigned int> distribution(0,ideal_vertices-1);

	unsigned int obstalce_coount = std::round(ideal_vertices * coverage);

}

/// @brief This function is expected to check whether the edge between two points intersects an obstacle 
/// @param p1 Struct containing the coordinates for the first point
/// @param p2 Struct containing the coordinates for the second point
/// @return ObstacleAnalysis struct containing a boolean - whether an obstacle was intersecte, and a value indicating the proximity to the object (if not intersected) 
ObstacleAnalysis check_obstacle(Point p1, Point p2){
	// Return variable
	ObstacleAnalysis result;

	

	return result;
}



/*
* Obstalces
* User needs to define ther obstacles in absolute coordinates 
* Meaning they need to provide coordinates in their units (m) 
* My function will convert graph cell coordinates to to user coordinates using the resolution factor
* User will be passed the coordinates of the vertices in their desired units to check 
* User checks whether an obstacle is intersected however they want to
* User returns a struct containing whether an obstacle is intersected (bool) and a proximity value (in their units -- only if the obstacle is not intersected)
* 
*/
