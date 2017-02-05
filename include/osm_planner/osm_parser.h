//
// Created by michal on 31.12.2016.
//

#ifndef PROJECT_OSM_PARSER_H
#define PROJECT_OSM_PARSER_H


#include <tinyxml.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>



class OsmParser{
public:

    typedef struct osm_node {
        double latitude;
        double longitude;
    } OSM_NODE;

    typedef struct osm_node_with_id{
        int id;
        OSM_NODE node;
    }OSM_NODE_WITH_ID;

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
    std::vector< std::vector<float> > * getGraphOfVertex(); //for dijkstra algorithm
    int getNearestPoint(double lat, double lon); //return OSM node ID
    int getNearestPointXY(double point_x, double point_y); //return OSM node ID
    OSM_NODE getNodeByID(int id);                //OSM NODE contains geogpraphics coordinates

    //SETTERS
    void setStartPoint(double latitude, double longitude); //set the zero point in cartezian coordinates

    //Embedded class for calculating distance and bearing
    class Haversine{

        //Functions was inspired by: http://www.movable-type.co.uk/scripts/latlong.html
    public:
        static double getDistance(OsmParser::OSM_NODE node1, OsmParser::OSM_NODE node2);
        static double getCoordinateX(double lon1, double lon2, double lat1, double lat2);
        static double getCoordinateX(OsmParser::OSM_NODE node1, OsmParser::OSM_NODE node2);
        static double getCoordinateY(double lat1, double lat2);
        static double getCoordinateY(OsmParser::OSM_NODE node1, OsmParser::OSM_NODE node2);
        static double getBearing(OsmParser::OSM_NODE node1, OsmParser::OSM_NODE node2);

    private:
        const static double R = 6371e3;
        const static double DEG2RAD = M_PI/180;
        const static double RAD2DEG = 180/M_PI;
    };

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
    std::vector <OSM_NODE_WITH_ID> interpolated_nodes;
    std::vector <TRANSLATE_TABLE> table;
    std::vector <std::vector <float> > networkArray;

    //start point - must be set and than you can publishing paths
    OSM_NODE startPoint;

    void createMarkers();

    void createWays(TiXmlHandle* hRootWay, TiXmlHandle *hRootNode, std::string osm_key, std::string osm_value);
    void createNodes(TiXmlHandle *hRootNode);
    void createNetwork(double interpolate_distance);
    void getNodesInWay(TiXmlElement* wayElement, OSM_WAY *way, std::vector<OSM_NODE_WITH_ID> nodes);
    bool translateID(int id, int *ret_value);

    //ADDED for interpolation
    //------------------------------------------
    //interpolation - creating more nodes between parameters node1 and node2
    std::vector<OSM_NODE> getInterpolatedNodes(OSM_NODE node1, OSM_NODE node2);
    double interpolation_max_distance;
    //finding node by osm id in std::vector<OSM_NODE_WITH_ID> nodes buffer
    OSM_NODE_WITH_ID getNodeByOsmId(std::vector<OSM_NODE_WITH_ID> nodes, int id);
    //------------------------------------------

};


#endif //PROJECT_OSM_PARSER_H
