//
// Created by michal on 11/11/18.
//

#ifndef PROJECT_PATH_FINDER_BASE_H
#define PROJECT_PATH_FINDER_BASE_H

#include <exception>
#include <memory>
#include <vector>

namespace osm_planner {

    namespace path_finder_algorithm {

        //! OSM PathFinderBase class
        /*!
         * The PathFinderBase class is a virtual base class, for finding shortest path on the osm map
         * Algorithms should be use orientation graph for planning
        */
        class PathFinderBase {
        public:

            /**
             * @brief Empty constructor
             */
            PathFinderBase() {};

            /**
             * @brief Start find shortest path
             * @param graph - A pointer to map represented as orientation graph in 2D array.
             * In array is defined distances from every node to every node. If distance is negative,
             * then the node has no connect with second node.
             * Example: graph[0][1] = 1, means, that from node[0] is connected with node[1] and distances between node is 1m
             * @param source - Id of node, where is start
             * @param target - Id of node, where is goal
             * @return A array of node ID's, which represent a shortest path on the osm map
             * @throw path_finder_algorithm::PathFinderException if isn't possible to find path
             */
            virtual std::vector<int> findShortestPath(std::shared_ptr<std::vector<std::vector<float>>> graph, int source, int target) = 0;
        };

        //! PathFinderException
        /*!
         * The PathFinderException for throwing exception, when path planner fails.
        */
        class PathFinderException : public std::exception {
        public:

            const static int NO_PATH_FOUND = 1;

            /**
             * @brief Exception constructor
             * @param err_id - error code id
             */
            PathFinderException(int err_id) {
                this->err_id_ = err_id;
            }

            /**
             * @brief std::exception override
             * @return no path found message
             */
            virtual const char *what() const throw() {
                return "No path found";
            }

            /**
             * @brief Get error id for error handling
             * @return A error code
             */
            int getErrId() {
                return err_id_;
            }

        private:
            int err_id_;
        };
    }
}
#endif //PROJECT_PATH_FINDER_BASE_H
