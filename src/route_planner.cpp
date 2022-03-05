#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {

    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    start_node = &m_Model.FindClosestNode( start_x, start_y );

    end_node = &m_Model.FindClosestNode( end_x, end_y );

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

  return end_node->distance( *node );

}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

   current_node->FindNeighbors();

   for ( auto neighbor: current_node->neighbors ) {

     neighbor->parent = current_node;
     neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
     //neighbor->g_value = neighbor->g_value + current_node->distance( *neighbor ); //current_node->g_value + 0.1;
     neighbor->h_value = CalculateHValue( neighbor );

     open_list.push_back( neighbor );
     neighbor->visited = true;

   }

}

/* The old way prev to c++ lambda
bool compare( RouteModel::Node *a, RouteModel::Node *b ) {

   auto a_total = a->h_value + a->g_value;
   auto b_total = b->h_value + b->g_value;

   return a_total > b_total;

}
*/

RouteModel::Node *RoutePlanner::NextNode() {

  sort( open_list.begin(), open_list.end(), []( const RouteModel::Node *a, const RouteModel::Node *b ) {

    auto a_total = a->h_value + a->g_value;
    auto b_total = b->h_value + b->g_value;

    return a_total > b_total;

  } );

  auto next_node = open_list.back();

  open_list.pop_back();

  return next_node;

}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while ( current_node->parent != nullptr ) { //I need to check the ->parent. Because in the next line i defer the pointer *(current_node->parent). 

      //if current_node->parent is nullptr crash the program
      distance += current_node->distance( *(current_node->parent) );

      path_found.insert( path_found.begin(), *current_node );

      current_node = current_node->parent;

    }
    
    path_found.insert( path_found.begin(), *current_node );

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {

    RouteModel::Node *current_node = nullptr; //start_node;
    start_node->visited = true;
    open_list.push_back( start_node );

    while ( open_list.size() > 0 ) {

      current_node = NextNode();
      
      if ( current_node->x == end_node->x &&
           current_node->y == end_node->y ) {

        m_Model.path = ConstructFinalPath( current_node );
        break;

      }

      AddNeighbors( current_node ); 

    };

}