
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <utility>
#include "HEAR_core/Block.hpp"

namespace HEAR
{

// Data structure to store a graph edge
struct Edge {
   std::pair<int, int> src,  dest;
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
 
        // add edges to the directed graph
        for (auto &edge: edges)
        {
            // insert at the end
            adjList[edge.src.first].push_back(Connection{ edge.src.second, edge.dest.second, edge.dest.first});
 
            // uncomment the following code for undirected graph
            // adjList[edge.dest].push_back(edge.src);
        }
    }
    
    Graph(){}
    
};

}

#endif
