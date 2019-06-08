//
// Created by michal on 31.12.2016.
//

#ifndef PROJECT_OSM_PARSER_H
#define PROJECT_OSM_PARSER_H


//parser
#include <tinyxml.h>

//ros and tf
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

//messages
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>

#include <osm_planner/coordinates_converters/coordinates_converter_base.h>

namespace osm_planner {

    class Parser {
    public:

        typedef struct osm_node {
            double latitude;
            double longitude;
            double altitude;
            double angle;
        } OSM_NODE;

        typedef struct osm_node_with_id {
            int id;
            OSM_NODE node;
        } OSM_NODE_WITH_ID;

        typedef struct osm_way {
            int id;
            std::vector<int> nodesId;
        } OSM_WAY;

        typedef struct translate_table {
            int oldID;
            int newID;
        } TRANSLATE_TABLE;


        class NodeWithDistance : public std::pair<int, double>{
        public:
            NodeWithDistance(int id, double dist){first = id; second = dist;}
            bool operator<(const NodeWithDistance& rhs) const {
                return second < rhs.second;
            }
        };

        const static int CURRENT_POSITION_MARKER = 0;
        const static int TARGET_POSITION_MARKER = 1;

        Parser(std::string xml);

        Parser();

        //start parsing
        void parse(bool onlyFirstElement = false);

        //publishing functions
        void publishPoint(int pointID, int marker_type, double radius, geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(0));
        void publishPoint(geometry_msgs::Point point, int marker_type, double radius, geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(0));
        void publishPoint(const OSM_NODE &node, int marker_type, double radius, geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(0));
        void publishPoint(double latitude, double longitude, int marker_type, double radius, geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(0));

        void publishRouteNetwork();

        void publishRefusedPath(std::vector<int> nodesInPath);

        //deleting edge on the graph
        void deleteEdgeOnGraph(int nodeID_1, int nodeID_2);

        //GETTERS
        std::shared_ptr<std::vector<std::vector<float>>> getGraphOfVertex(); //for dijkstra algorithm
        int getNearestPoint(double lat, double lon); //return OSM node ID
        int getNearestPointXY(double point_x, double point_y); //return OSM node ID
        std::set<NodeWithDistance> getNearestPoints(double point_x, double point_y, int size); //return OSM node ID

        OSM_NODE getNodeByID(int id);                //OSM NODE contains geogpraphics coordinates
        nav_msgs::Path getPath(std::vector<int> nodesInPath); //get the XY coordinates from vector of IDs
        std::string getMapFrameName();
        double getInterpolationMaxDistance();

        //SETTERS
        void setStartPoint(double latitude, double longitude, double bearing); //set the zero point in cartezian coordinates
        void setStartPoint(double latitude, double longitude); //set the zero point in cartezian coordinates
        void setStartPoint(const OSM_NODE node); //set the zero point in cartezian coordinates
        void setStartPoint(int id);
        void setRandomStartPoint();
        void setNewMap(std::string xml);
        void setTypeOfWays(std::vector<std::string> types);

        void setInterpolationMaxDistance(double param);

        std::shared_ptr<coordinates_converters::CoordinatesConverterBase> getCalculator();         //get Distance and bearing calculator


    private:

        //distance and bearing calculator
        std::shared_ptr<coordinates_converters::CoordinatesConverterBase> coordinatesConverter;

        //map source
        std::string xml;

        std::vector<std::string> types_of_ways;

        //publishers
        ros::Publisher position_marker_pub;
        ros::Publisher target_marker_pub;
        ros::Publisher path_pub;
        ros::Publisher refused_path_pub;

        //visualization msgs
        visualization_msgs::Marker position_marker, target_marker;

        std::string map_frame; //name of frame for msgs

        int size_of_nodes;  //usage in function getNodesInWay(), counter of currently read nodes

        //vector arrays of OSM nodes and ways
        std::vector<OSM_WAY> ways;
        std::vector<OSM_NODE> nodes;
        std::vector<OSM_NODE_WITH_ID> interpolated_nodes;
        std::vector<TRANSLATE_TABLE> table;
        std::vector<std::vector<float> > networkArray;

       void initialize();

        void createMarkers();

        void createWays(TiXmlHandle *hRootWay, TiXmlHandle *hRootNode, std::vector<std::string> osm_value, bool onlyFirstElement = false);
        bool isSelectedWay(TiXmlElement *tag, std::vector<std::string> values);

        void createNodes(TiXmlHandle *hRootNode, bool onlyFirstElement = false);

        void createNetwork();

        void getNodesInWay(TiXmlElement *wayElement, OSM_WAY *way, std::vector<OSM_NODE_WITH_ID> nodes);

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
}

#endif //PROJECT_OSM_PARSER_H
