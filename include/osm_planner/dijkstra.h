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

    Dijkstra(std::vector <std::vector<double> > graph);
    std::vector<int> findShortestPath(int src, int target);
    std::vector<int> getSolution(int target);

private:

    std::vector <std::vector<double> > graph; //matrix representation of the graph
    std::vector<int> parent;    // Parent array to store shortest path tree
    std::vector<double> dist;   // The output array. dist[i] will hold
                                // the shortest distance from src to i
    std::vector<int> path;      // The shortest path - inicialize in function getShortestPath()
    int source;                 //start point

    int minDistance(std::vector<double> dist, bool sptSet[]);
    void printPath(std::vector<int> parent, int j);
};

#endif //PROJECT_DIJKSTRA_H
