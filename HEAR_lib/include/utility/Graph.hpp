
#pragma once

#include <vector>
#include <utility>
 
// Data structure to store a graph edge
struct Edge {
    std::pair<int, int> src, dest;
};
 
// A class to represent a graph object
class Graph
{
public:
    // a vector of vectors to represent an adjacency list
    std::vector<std::vector<std::vector<int>>> adjList;
 
    // Graph Constructor
    Graph(std::vector<Edge> const &edges, int N);
    
};
 
