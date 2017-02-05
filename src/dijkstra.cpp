//
// Created by michal on 30.12.2016.
//
#include <osm_planner/dijkstra.h>

// Funtion that implements Dijkstra's single source shortest path
// algorithm for a graph represented using adjacency matrix
// representation
Dijkstra::Dijkstra(){
}

/*void Dijkstra::setGraph(std::vector< std::vector<double> > graph){

//    this->graph = graph;
}*/

std::vector<int> Dijkstra::findShortestPath(std::vector <std::vector<float> > graph, int src, int target){


                                                //graph - matrix representation of the graph
    std::vector <int> parent(graph.size());     // Parent array to store shortest path tree
    std::vector <float> dist(graph.size());    // The output array. dist[i] will hold
                                                // the shortest distance from src to i

    this->source = src;
    // sptSet[i] will true if vertex i is included / in shortest
    // path tree or shortest distance from src to i is finalized
    bool sptSet[graph.size()];

    // Initialize all distances as INFINITE and stpSet[] as false
    for (int i = 0; i < graph.size(); i++)
    {
       //parent[0] = -1;
        parent[i] = -1;
        dist[i] = 1000.0; //INT_MAX
        sptSet[i] = false;
    }

    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < graph.size()-1; count++)
    {
        // Pick the minimum distance vertex from the set of
        // vertices not yet processed. u is always equal to src
        // in first iteration.
        int u = minDistance(dist, sptSet);

        // Mark the picked vertex as processed
        sptSet[u] = true;

        // Update dist value of the adjacent vertices of the
        // picked vertex.
        for (int v = 0; v < graph.size(); v++)

            // Update dist[v] only if is not in sptSet, there is
            // an edge from u to v, and total weight of path from
            // src to v through u is smaller than current value of
            // dist[v]
            if (!sptSet[v] && graph[u][v] &&
                dist[u] + graph[u][v] < dist[v])
            {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];

            }
    }

    // print the constructed distance array
    //printf("\ngetting solution\n");
    return getSolution(parent, dist, target);
    //printSolution(dist, parent);
}


int Dijkstra::minDistance(std::vector<float> dist, bool sptSet[])
{
    // Initialize min value
    int min = 1000.0, min_index; //INT_MAX

    for (int v = 0; v < dist.size(); v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}

// Function to print shortest path from source to j
// using parent array
void Dijkstra::printPath(std::vector<int> parent, int j)
{
    // Base Case : If j is source
    if (parent[j]==-1)
        return;

   //printf("\nprintf path %d\n", j);

    //todo zistit ako to tu funguje pada to tu
    printPath(parent, parent[j]);

    path.push_back(j);
}

// A utility function to print the constructed distance
// array

std::vector<int> Dijkstra::getSolution(std::vector<int> parent, std::vector<float> dist,  int target) {

    path.clear();


    if (dist[target] >= 1000.0) {
        return path;
    }
   path.push_back(source);

    // for (int i = target; parent[i] != -1; i = parent[i]){
  //  }

    printPath(parent, target);

    return path;
}

std::vector<int> Dijkstra::getSolution() {

    return path;
}