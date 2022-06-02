#pragma once

/*
 * Copyright (C) 2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */


#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/internal/Graph.h>


/**
 * \brief This class serves as a method of exposing the internal protected members of the 
 *        lanelet2 RoutingGraph. This allows for a conversion method to be written which converts the routing
 *        graph to a ROS message representation. 
 * 
 *  ASSUMPTION: Note this class is heavily dependant on the non-public implementation API of lanelet2 (v1.1.1).
 *              Any change to the underlying data structures may negatively impact this classes performance or ability to compile. 
 */ 
class RoutingGraphAccessor : public lanelet::routing::RoutingGraph {

  public:

  /**
   * \brief Returns a ROS message version of this RoutingGraph. This is done by accessing protected data members directly
   * 
   * \param participant The participant which was used to build this routing graph. 
   *                    NOTE: This is not a choice. This must match the value used to generate this object.
   * 
   * \return The ros message which can be used to regenerate this routing graph structure. 
   */ 
  autoware_lanelet2_msgs::msg::RoutingGraph routingGraphToMsg(const std::string& participant) {


    autoware_lanelet2_msgs::msg::RoutingGraph msg; // output message

    // Assign base fields
    msg.num_unique_routing_cost_ids = this->graph_->numRoutingCosts();
    msg.participant_type = participant;

    // Get the underlying graph
    // Type will be boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexInfo, EdgeInfo>
    // As version of lanelet2 used when writing this function
    auto underlying_graph = this->graph_->get();

    // Iterate over all vertices in the graph
    boost::graph_traits<lanelet::routing::internal::GraphType>::vertex_iterator vi, vi_end;
    
    for(boost::tie(vi, vi_end) = boost::vertices(underlying_graph); vi != vi_end; ++vi) {
       
      autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges vertex_and_edges;

      lanelet::routing::internal::GraphType::vertex_descriptor source = *vi;
      
      // Iterate over all outgoing edges for this vertex
      boost::graph_traits<lanelet::routing::internal::GraphType>::out_edge_iterator ei, ei_end;
      for(boost::tie(ei, ei_end) = boost::out_edges(source, underlying_graph); ei != ei_end; ++ei) {
       
        lanelet::routing::internal::GraphType::edge_descriptor edge = *ei;
        lanelet::routing::internal::GraphType::vertex_descriptor target = boost::target(edge, underlying_graph);


        // Populate the edge info in the message
        vertex_and_edges.lanelet_or_area_ids.push_back(underlying_graph[target].laneletOrArea.id());

        vertex_and_edges.edge_routing_costs.push_back(underlying_graph[edge].routingCost);
        vertex_and_edges.edge_routing_cost_source_ids.push_back(underlying_graph[edge].costId);

        uint8_t relation = 0;
        switch(underlying_graph[edge].relation) {
          case lanelet::routing::RelationType::Successor:
            relation = autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_SUCCESSOR; break;
          case lanelet::routing::RelationType::Left:
            relation = autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_LEFT; break;
          case lanelet::routing::RelationType::Right:
            relation = autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_RIGHT; break;
          case lanelet::routing::RelationType::AdjacentLeft:
            relation = autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_ADJACENT_LEFT; break;
          case lanelet::routing::RelationType::AdjacentRight:
            relation = autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_ADJACENT_RIGHT; break;
          case lanelet::routing::RelationType::Conflicting:
            relation = autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_CONFLICTING; break;
          case lanelet::routing::RelationType::Area:
            relation = autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_AREA; break;
          default: // None relation will be default
            relation = autoware_lanelet2_msgs::msg::RoutingGraphVertexAndEdges::RELATION_NONE; break;
        }

        vertex_and_edges.edge_relations.push_back(relation);

      }

      vertex_and_edges.lanelet_or_area = underlying_graph[source].laneletOrArea.id();

      // Assign core vertext to the correct array in the ros message based on its type
      if (underlying_graph[source].laneletOrArea.isLanelet()) { // Lanelet
        msg.lanelet_vertices.push_back(vertex_and_edges);

      } else { // Area
        msg.area_vertices.push_back(vertex_and_edges);
      }
      
    }

    return msg;
  }
};

