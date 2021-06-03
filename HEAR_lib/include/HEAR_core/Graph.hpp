
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <utility>
#include <iostream>
#include "HEAR_core/Block.hpp"

namespace HEAR
{

// Data structure to store a graph edge
struct Edge {
    int src_block_idx;
    int src_port;
    int dest_port;
    int dest_block_idx;
};

struct Connection {
    int src_port;
    int dest_port;
    int dest_block_idx;
};
 
// A class to represent a graph object
class Graph
{
public:
    // a vector of vectors to represent an adjacency list
    std::vector<std::vector<Connection>> adjList;
 
    // Graph Constructor
    Graph(std::vector<Edge> const &edges, int N){
        // resize the vector to hold `N` elements of type `vector<int>`
        adjList.resize(N);
//        std::cout <<"num_edges" <<edges.size() <<std::endl;
        // add edges to the directed graph
        for (auto &edge: edges)
        {
            // insert at the end
            Connection con;
            con.dest_block_idx = edge.dest_block_idx;
            con.dest_port = edge.dest_port;
            con.src_port = edge.src_port;
            adjList[edge.src_block_idx].push_back(con);
 
            // uncomment the following code for undirected graph
            // adjList[edge.dest].push_back(edge.src);
        }
    }
    
    Graph(){}
    
};

}

#endif
