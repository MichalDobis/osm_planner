//
// Created by michal on 31.12.2016.
// Modify and refactor by michal on 11.11.2018
//

#ifndef PROJECT_DIJKSTRA_H
#define PROJECT_DIJKSTRA_H

#include "osm_planner/path_finder_algorithm/path_finder_base.h"

namespace osm_planner {

    namespace path_finder_algorithm {

        //! Dijkstra class
        /*!
         * Dijsktra algorithm on orientation graph.
         * A C / C++ program for Dijkstra's single source shortest
         * Algorithms should be use orientation graph for planning
         * path algorithm. The program is for adjacency matrix
         * representation of the graph.
         * A utility function to find the vertex with minimum distance
         * value, from the set of vertices not yet included in shortest
         * path tree
         * Inspired by http://www.geeksforgeeks.org/greedy-algorithms-set-6-dijkstras-shortest-path-algorithm/
        */
        class Dijkstra : public PathFinderBase {
        public:

            /**
             * @brief Empty constructor
             */
            Dijkstra();

            /**
             * @brief Implementation of method from PathFinderBase class
             */
            std::vector<int> findShortestPath(std::shared_ptr<std::vector<std::vector<float>>> graph, int source, int target) override;

        private:

            /**
             * @brief Method for finding vertex with minimal distance
             * @param dist - reference to final path array
             * @param sptSet - shortest path tree
             * @return A index of vertex, when is minimal distance
             */
            int minDistance(const std::vector<float> &dist, bool sptSet[]);

            /**
             * @brief Method for parsing solution. From shortest path three
             * are selected data into array contained vertex id's
             * @param parent - reference to final path array
             * @param dist - shortest path tree
             * @param source - Start osm node
             * @param target - Goal osm node
             * @return A shortest path
             */
            std::vector<int> getSolution(const std::vector<int> &parent, const std::vector<float> &dist, int source, int target);

            /**
             * @brief Function to set shortest path from source to j
             * using parent array
             * Here is used recursive call of method
             * @param path - reference to final path array
             * @param parent - shortest path tree
             * @param j - If j is source then return from method
             */
            void setPath(std::vector<int> &path, const std::vector<int> &parent, int j);
        };
    }
}

#endif //PROJECT_DIJKSTRA_H
