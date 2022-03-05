#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    //auto temp = m_Model.FindClosestNode( end_x, end_y );
    
    start_node = &m_Model.FindClosestNode( start_x, start_y );

    end_node = &m_Model.FindClosestNode( end_x, end_y ); //new RouteModel::Node();

    //start_node->h_value = CalculateHValue( start_node );

    /*
    end_node->x = temp.x;
    end_node->y = temp.y;
    end_node->g_value = 0;
    end_node->h_value = temp.h_value;
    */


    //std::cout << "g: " << temp.g_value << ",h:" << temp.h_value << std::endl;

    /*
    start_node = new RouteModel::Node();
    start_node->x = temp.x;
    start_node->y = temp.y;
    start_node->g_value = 0;
    start_node->h_value = CalculateHValue( start_node );
    */

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

  return end_node->distance( *node );

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

   current_node->FindNeighbors();

   //open_list.push_back( current_node );
   //std::cout << "x: " << current_node->x << std::endl;
   //std::cout << "y: " << current_node->y << std::endl;

   for ( auto neighbor: current_node->neighbors ) {

     neighbor->parent = current_node;
     neighbor->g_value = neighbor->g_value + current_node->distance( *neighbor ); //current_node->g_value + 0.1;
     neighbor->h_value = CalculateHValue( neighbor );

     open_list.push_back( neighbor );
     neighbor->visited = true;

   }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool compare( RouteModel::Node *a, RouteModel::Node *b ) {

   auto a_total = a->h_value + a->g_value;
   auto b_total = b->h_value + b->g_value;

   return a_total > b_total;

}

RouteModel::Node *RoutePlanner::NextNode() {

  sort( open_list.begin(), open_list.end(), compare );

  /*
  for ( auto node : open_list ) {

    std::cout << "h_value + g_value: " << node->h_value + node->g_value << std::endl;

  }
  */

  auto next_node = open_list.back();

  //std::cout << "next_node h_value + g_value: " << next_node->h_value + next_node->g_value << std::endl;

  open_list.pop_back();

  return next_node;

}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *temp = current_node;

    //int run = 0;

    // TODO: Implement your solution here.
    while ( temp != nullptr ) {

      //std::cout << "g_value + h_value: " << temp->g_value + temp->h_value << " run: " << run++ << std::endl;

      distance += temp->g_value;

      path_found.insert( path_found.begin(), *temp );

      temp = temp->parent;

    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {

    RouteModel::Node *current_node = nullptr; //start_node;
    start_node->visited = true;
    open_list.push_back( start_node );
    //int run = 0;
    //AddNeighbors( current_node ); 

    // TODO: Implement your solution here.
    while ( open_list.size() > 0 ) {

      //std::cout << run++ << std::endl;
      //std::cout << "open_list.size: " << open_list.size() << std::endl;

      current_node = NextNode();

      //if ( run == 2 ) {

      //  m_Model.path = ConstructFinalPath( current_node );
                
      //  break;

      //}

      
      if ( current_node->x == end_node->x &&
           current_node->y == end_node->y ) {

        //std::cout << "found" << std::endl;
        m_Model.path = ConstructFinalPath( current_node );
        break;

      }

      //if ( current_node->visited == false ) {

      AddNeighbors( current_node ); 

      //}

    };

}