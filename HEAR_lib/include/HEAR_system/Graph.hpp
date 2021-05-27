
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
    Graph();
    
};
 
Graph::Graph(std::vector<Edge> const &edges, int N){
        // resize the vector to hold `N` elements of type `vector<int>`
        adjList.resize(N);
 
        // add edges to the directed graph
        for (auto &edge: edges)
        {
            // insert at the end
            adjList[edge.src.first].push_back({ edge.src.second, edge.dest.first, edge.dest.second});
 
            // uncomment the following code for undirected graph
            // adjList[edge.dest].push_back(edge.src);
        }
    }