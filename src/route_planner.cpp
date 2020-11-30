#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Calculates the h value to the end_node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

// Expands the current node by adding all unvisited neighbor nodes to the open list. 
// In doing so its sets the correspondung neighbor nodes visted attribute to true and set the corresponding
// parent, h_value and g_value.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
 
  current_node->FindNeighbors();

  for (RouteModel::Node* node : current_node->neighbors){
    node->parent = current_node;
    node->g_value = current_node->g_value + current_node->distance(*node);
    node->h_value = CalculateHValue(node);
    this->open_list.push_back(node);
    node->visited = true;
  }
}


// Compares the sum of the h and g value of two nodes 
bool RoutePlanner::Compare(const RouteModel::Node *a, const RouteModel::Node *b){
  float f1 = a->g_value + a->h_value;
  float f2 = b->g_value + b->h_value;
  return f1 > f2; 
}

// Sorts the open_list according to the sum of the h value and g value and returns a
// pointer to the node in the list with the lowest sum.
RouteModel::Node *RoutePlanner::NextNode(){
  // Order open list in descending order.
  std::sort(open_list.begin(), open_list.end(), Compare);
  RouteModel::Node* lowest_cost_node = open_list.back();
  open_list.pop_back();
  return lowest_cost_node;
}

// Returns the final path found from the A* search algorithms
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node *iterator_node = current_node ;  
    path_found.insert(path_found.begin(),*current_node);

    // TODO: Implement your solution here.
    bool start_found = false;
    while(start_found == false){
      distance += iterator_node->distance(*(iterator_node->parent));
      iterator_node = iterator_node->parent;
      path_found.insert(path_found.begin(),*iterator_node);
      if(iterator_node == start_node){
        start_found = true;
      }

    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

// Implementation of the A* Algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    current_node = start_node;
    AddNeighbors(current_node);
    start_node->visited = true;

    bool found_route = false;
    while(found_route == false){
      current_node = NextNode();
      if(current_node == end_node){
        m_Model.path = ConstructFinalPath(current_node);
        found_route = true;
      }
      AddNeighbors(current_node);
    }
}
