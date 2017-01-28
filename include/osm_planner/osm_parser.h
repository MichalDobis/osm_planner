//
// Created by michal on 31.12.2016.
//

#ifndef PROJECT_OSM_PARSER_H
#define PROJECT_OSM_PARSER_H


#include <tinyxml.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

class OsmParser {
public:

    typedef struct osm_node {
        int id;
        double latitude;
        double longitude;
    } OSM_NODE;

    typedef struct osm_way {
        int id;
        std::vector<int> nodesId;
    } OSM_WAY;

    typedef struct translate_table {
        int oldID;
        int newID;
    } TRANSLATE_TABLE;

    const static int CURRENT_POSITION_MARKER = 0;
    const static int TARGET_POSITION_MARKER = 1;

    OsmParser(std::string xml);

    //publishing functions
    void publishPoint(int pointID, int marker_type);
    void publishPoint(double latitude, double longitude, int marker_type);
    void publishRouteNetwork();
    void publishRefusedPath(std::vector<int> nodesInPath);
    void publishPath(std::vector<int> nodesInPath, double target_lat, double target_lon);

    //deleting edge on the graph
    void deleteEdgeOnGraph(int nodeID_1, int nodeID_2);

    //GETTERS
    std::vector< std::vector<double> > getGraphOfVertex(); //for dijkstra algorithm
    int getNearestPoint(double lat, double lon); //return OSM node ID
    OSM_NODE getNodeByID(int id);                //OSM NODE contains geogpraphics coordinates
    double getDistance(OSM_NODE node1, OSM_NODE node2);
    double getBearing(OSM_NODE node1, OSM_NODE node2);

    //SETTERS
    void setStartPoint(double latitude, double longitude); //set the zero point in cartezian coordinates

private:

    //publishers
    ros::Publisher position_marker_pub;
    ros::Publisher target_marker_pub;
    ros::Publisher path_pub;
    ros::Publisher refused_path_pub;
    ros::Publisher shortest_path_pub;

    //visualization msgs
    visualization_msgs::Marker position_marker, target_marker;

    //msgs for shortest path
    nav_msgs::Path sh_path;

    std::string map_frame; //name of frame for msgs

    bool visualization; //enable or disable publishing markers and paths for rviz visualization

    //vector arrays of OSM nodes and ways
    std::vector <OSM_WAY> ways;
    std::vector <OSM_NODE> nodes;
    std::vector <TRANSLATE_TABLE> table;
    std::vector <std::vector <double> > networkArray;

    //start point - must be set and than you can publishing paths
    OSM_NODE startPoint;

    void createMarkers();

    void createWays(TiXmlHandle* hRootWay);
    void createNodes(TiXmlHandle *hRootNode);
    void createNetwork();
    void getNodesInWay(TiXmlElement* wayElement, OSM_WAY *way);
    bool translateID(int id, int *ret_value);
};
#endif //PROJECT_OSM_PARSER_H
