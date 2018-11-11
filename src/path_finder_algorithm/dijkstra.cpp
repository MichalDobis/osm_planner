//
// Created by michal on 30.12.2016.
// Modify and refactor by michal on 11.11.2018
//

#include "osm_planner/path_finder_algorithm/dijkstra.h"

namespace osm_planner {

    namespace path_finder_algorithm {

        Dijkstra::Dijkstra() : PathFinderBase() {}

        std::vector<int> Dijkstra::findShortestPath(std::shared_ptr<std::vector<std::vector<float>>> graph, int source, int target) {

            std::vector<int> parent((*graph).size());     // Parent array to store shortest path tree
            std::vector<float> dist((*graph).size());    // The output array. dist[i] will hold

            // sptSet[i] will true if vertex i is included / in shortest
            // path tree or shortest distance from source to i is finalized
            bool sptSet[(*graph).size()];

            // Initialize all distances as INFINITE and stpSet[] as false
            for (int i = 0; i < (*graph).size(); i++) {
                //parent[0] = -1;
                parent[i] = -1;
                dist[i] = 1000.0; //INT_MAX
                sptSet[i] = false;
            }

            // Distance of source vertex from itself is always 0
            dist[source] = 0;

            // Find shortest path for all vertices
            for (int count = 0; count < (*graph).size() - 1; count++) {
                // Pick the minimum distance vertex from the set of
                // vertices not yet processed. u is always equal to source
                // in first iteration.
                int u = minDistance(dist, sptSet);

                // Mark the picked vertex as processed
                sptSet[u] = true;

                // Update dist value of the adjacent vertices of the
                // picked vertex.
                for (int v = 0; v < (*graph).size(); v++)

                    // Update dist[v] only if is not in sptSet, there is
                    // an edge from u to v, and total weight of path from
                    // source to v through u is smaller than current value of
                    // dist[v]
                    if (!sptSet[v] && (*graph)[u][v] &&
                        dist[u] + (*graph)[u][v] < dist[v]) {
                        parent[v] = u;
                        dist[v] = dist[u] + (*graph)[u][v];
                    }

                // print the constructed distance array
                // TODO toto je asi bug, algoritmus by mal pokracovat kym neprehlada cely priestor
                if (u == target) return getSolution(parent, dist, source, target);
            }

            // print the constructed distance array
            return getSolution(parent, dist, source, target);
        }

        int Dijkstra::minDistance(const std::vector<float> &dist, bool sptSet[]) {

            // Initialize min value
            int min = 1000.0, min_index; //INT_MAX
            for (int v = 0; v < dist.size(); v++)
                if (sptSet[v] == false && dist[v] <= min)
                    min = dist[v], min_index = v;
            return min_index;
        }

        void Dijkstra::setPath(std::vector<int> &path, const std::vector<int> &parent, int j) {

            // Base Case : If j is source
            if (parent[j] == -1)
                return;

            setPath(path, parent, parent[j]);
            path.push_back(j);
        }

        std::vector<int> Dijkstra::getSolution(const std::vector<int> &parent, const std::vector<float> &dist, int source, int target) {

            std::vector<int> path;
            path.clear();

            if (dist[target] >= 1000.0) {
                throw PathFinderException(PathFinderException::NO_PATH_FOUND);
            }

            path.push_back(source);
            setPath(path, parent, target);
            return path;
        }
    }
}