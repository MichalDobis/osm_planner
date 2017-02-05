//
// Created by michal on 31.12.2016.
/**
 * Inspired by http://www.geeksforgeeks.org/greedy-algorithms-set-6-dijkstras-shortest-path-algorithm/
 *
 * */
//

#ifndef PROJECT_DIJKSTRA_H
#define PROJECT_DIJKSTRA_H

// A C / C++ program for Dijkstra's single source shortest
// path algorithm. The program is for adjacency matrix
// representation of the graph.
#include <stdio.h>
#include <limits.h>
#include <vector>

// A utility function to find the vertex with minimum distance
// value, from the set of vertices not yet included in shortest
// path tree

class Dijkstra{
public:

    Dijkstra();
    std::vector<int> findShortestPath(std::vector <std::vector<float> > * graph, int src, int target);
    std::vector<int> getSolution();

private:

    std::vector<int> path;      // The shortest path - initialize in function getShortestPath()
    int source;                 //start point

    int minDistance(std::vector<float> dist, bool sptSet[]);
    std::vector<int> getSolution(std::vector<int> parent, std::vector<float> dist, int target);
    void printPath(std::vector<int> parent, int j);
};

#endif //PROJECT_DIJKSTRA_H
