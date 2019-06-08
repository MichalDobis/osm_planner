//
// Created by michal on 31.12.2016.
//


#include <osm_planner/osm_parser.h>
#include <osm_planner/coordinates_converters/haversine_formula.h>

#include <osm_planner/coordinates_converters/wgs_84_elipsoid.h>

namespace osm_planner {


    Parser::Parser() {

      initialize();
      coordinatesConverter = std::make_shared<coordinates_converters::HaversineFormula>();
    }

    Parser::Parser(std::string file) : xml(file) {

        initialize();
        coordinatesConverter = std::make_shared<coordinates_converters::HaversineFormula>();

    }

   void Parser::initialize(){

        ros::NodeHandle n("~/Planner");

        //get the parameters
        n.param<std::string>("global_frame", map_frame, "/world");

       //Publishers for visualization
       position_marker_pub = n.advertise<visualization_msgs::Marker>("position_marker", 5);
       target_marker_pub = n.advertise<visualization_msgs::Marker>("target_marker", 1);

       path_pub = n.advertise<nav_msgs::Path>("route_network", 10);
       refused_path_pub = n.advertise<nav_msgs::Path>("refused_path", 10);

       createMarkers();
    }

    /**PUBLISHING FUNCTIONS**/

    void Parser::parse(bool onlyFirstElement) {

        ros::Time start_time = ros::Time::now();
        TiXmlDocument doc(xml);
        TiXmlNode *osm;
        TiXmlNode *node;
        TiXmlNode *way;

        TiXmlHandle hRootNode(0);
        TiXmlHandle hRootWay(0);

        bool loadOkay = doc.LoadFile();
        if (loadOkay) {
            ROS_INFO("OSM planner: loaded map: %s", xml.c_str());
        } else {
            ROS_ERROR("OSM planner: Failed to load file %s", xml.c_str());
            throw std::runtime_error("Failed to load xml");
        }

        osm = doc.FirstChildElement();
        node = osm->FirstChild("node");
        way = osm->FirstChild("way");
        TiXmlElement *nodeElement = node->ToElement();
        TiXmlElement *wayElement = way->ToElement();


        hRootNode = TiXmlHandle(nodeElement);
        hRootWay = TiXmlHandle(wayElement);


        createWays(&hRootWay, &hRootNode, types_of_ways, onlyFirstElement);
        createNodes(&hRootNode, onlyFirstElement);

        if (onlyFirstElement) return;

        createNetwork();
    }

    // TODO spravit const ref
    void Parser::publishPoint(geometry_msgs::Point point, int marker_type, double radius, geometry_msgs::Quaternion orientation) {

        point.z = 0;

        switch (marker_type) {

            case CURRENT_POSITION_MARKER:
                position_marker.points.clear();
                position_marker.scale.x = radius;
                position_marker.scale.y = radius;
                position_marker.pose.position = point;
                position_marker_pub.publish(position_marker);
                break;

            case TARGET_POSITION_MARKER:
                target_marker.points.clear();
                target_marker.scale.x = radius;
                target_marker.scale.y = radius;
                target_marker.pose.position = point;
                target_marker.pose.orientation = orientation;
                target_marker_pub.publish(target_marker);
                break;
            default:
                break;
        }
    }

    void Parser::publishPoint(double latitude, double longitude, int marker_type, double radius, geometry_msgs::Quaternion orientation) {

        geometry_msgs::Point point;
        point.x = 0;
        point.y = 0;

        OSM_NODE node;

        node.longitude =longitude;
        node.latitude = latitude;

        point.x = coordinatesConverter->getCoordinateX(node);
        point.y = coordinatesConverter->getCoordinateY(node);

        publishPoint(point, marker_type, radius, orientation);
    }


    void Parser::publishPoint(int pointID, int marker_type, double radius,geometry_msgs::Quaternion orientation) {

        geometry_msgs::Point point;
        point.x = coordinatesConverter->getCoordinateX(nodes[pointID]);
        point.y = coordinatesConverter->getCoordinateY(nodes[pointID]);
        publishPoint(point, marker_type, radius, orientation);
    }

    void Parser::publishPoint(const OSM_NODE &node, int marker_type, double radius,geometry_msgs::Quaternion orientation) {

        geometry_msgs::Point point;
        point.x = coordinatesConverter->getCoordinateX(node);
        point.y = coordinatesConverter->getCoordinateY(node);
        publishPoint(point, marker_type, radius, orientation);
    }

//publishing all paths
    void Parser::publishRouteNetwork() {

        nav_msgs::Path path;
        path.header.frame_id = map_frame;

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;

        for (int i = 0; i < ways.size(); i++) {
            path.poses.clear();

            for (int j = 0; j < ways[i].nodesId.size(); j++) {

                pose.pose.position.x = coordinatesConverter->getCoordinateX(nodes[ways[i].nodesId[j]]);
                pose.pose.position.y = coordinatesConverter->getCoordinateY(nodes[ways[i].nodesId[j]]);
                path.poses.push_back(pose);

            }

            usleep(90000);

            path.header.stamp = ros::Time::now();
            path_pub.publish(path);
        }
    }

//publishing defined path
    void Parser::publishRefusedPath(std::vector<int> nodesInPath) {

        nav_msgs::Path refused_path;
        refused_path.poses.clear();
        refused_path.header.frame_id = map_frame;

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;

        for (int i = 0; i < nodesInPath.size(); i++) {

            pose.pose.position.x = coordinatesConverter->getCoordinateX(nodes[nodesInPath[i]]);
            pose.pose.position.y = coordinatesConverter->getCoordinateY(nodes[nodesInPath[i]]);
            refused_path.poses.push_back(pose);
        }

        refused_path.header.stamp = ros::Time::now();
        refused_path_pub.publish(refused_path);
    }

    void Parser::deleteEdgeOnGraph(int nodeID_1, int nodeID_2) {

        networkArray[nodeID_1][nodeID_2] = 0;
        networkArray[nodeID_2][nodeID_1] = 0;

    }

    /* GETTERS */

//getter for dijkstra algorithm - getting only pointer for spare memory
    std::shared_ptr<std::vector<std::vector<float>>> Parser::getGraphOfVertex() {

        return std::make_shared<std::vector<std::vector<float>>>(networkArray);
    }

    //getting defined path
    //TODO std::shared_ptr
    nav_msgs::Path Parser::getPath(std::vector<int> nodesInPath) {

        //msgs for shortest path
        nav_msgs::Path sh_path;

        sh_path.poses.clear();
        sh_path.header.frame_id = map_frame;
        sh_path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose;

        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.header.frame_id = map_frame;

        for (int i = 0; i < nodesInPath.size(); i++) {

            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = coordinatesConverter->getCoordinateX(nodes[nodesInPath[i]]);
            pose.pose.position.y = coordinatesConverter->getCoordinateY(nodes[nodesInPath[i]]);
            double yaw = coordinatesConverter->getBearing(nodes[nodesInPath[i]]);

            pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            pose.header.seq = i;
            sh_path.poses.push_back(pose);
        }

        return sh_path;
    }

    std::string Parser::getMapFrameName() {
        return map_frame;
    }

    double Parser::getInterpolationMaxDistance() {
        return interpolation_max_distance;
    }


    int Parser::getNearestPoint(double lat, double lon) {

        OSM_NODE point;
        point.longitude = lon;
        point.latitude = lat;
        int id = 0;

        double distance = coordinatesConverter->getDistance(point, nodes[0]);
        double minDistance = distance;


        for (int i = 0; i < nodes.size(); i++) {
            distance = coordinatesConverter->getDistance(point, nodes[i]);

            if (minDistance > distance) {
                minDistance = distance;
                id = i;
            }
        }
        return id;
    }


    int Parser::getNearestPointXY(double point_x, double point_y) {

        int id = 0;

        double x = coordinatesConverter->getCoordinateX(nodes[0]);
        double y = coordinatesConverter->getCoordinateY(nodes[0]);
        double minDistance = sqrt(pow(point_x - x, 2.0) + pow(point_y - y, 2.0));

        for (int i = 0; i < nodes.size(); i++) {
            x = coordinatesConverter->getCoordinateX(nodes[i]);
            y = coordinatesConverter->getCoordinateY(nodes[i]);


            double distance = sqrt(pow(point_x - x, 2.0) + pow(point_y - y, 2.0));

            if (minDistance > distance) {
                minDistance = distance;
                id = i;
            }
        }
        return id;
    }

    std::set<Parser::NodeWithDistance> Parser::getNearestPoints(double point_x, double point_y, int size){
        std::set<NodeWithDistance> nearest_nodes;

        double x = coordinatesConverter->getCoordinateX(nodes[0]);
        double y = coordinatesConverter->getCoordinateY(nodes[0]);

        double minDistance = sqrt(pow(point_x - x, 2.0) + pow(point_y - y, 2.0));
        NodeWithDistance node(0, minDistance);
        nearest_nodes.insert(node);

        for (int i = 0; i < nodes.size(); i++) {
            x = coordinatesConverter->getCoordinateX(nodes[i]);
            y = coordinatesConverter->getCoordinateY(nodes[i]);


            double distance = sqrt(pow(point_x - x, 2.0) + pow(point_y - y, 2.0));
            NodeWithDistance next_node(i, distance);

            nearest_nodes.insert(next_node);

            if (nearest_nodes.size() > size){
                nearest_nodes.erase(std::prev(nearest_nodes.end()));
            }

        }
        return nearest_nodes;

    } //return OSM node ID


    //get distance and bearing calculator
    std::shared_ptr<coordinates_converters::CoordinatesConverterBase> Parser::getCalculator(){

        return coordinatesConverter;
    }

//return OSM NODE, which contains geographics coordinates
   Parser::OSM_NODE Parser::getNodeByID(int id) {

        return nodes[id];
    }

    /* SETTERS */

   void Parser::setStartPoint(double latitude, double longitude, double bearing) {

        coordinatesConverter->setOrigin(latitude, longitude);
        coordinatesConverter->setOffset(bearing);
   }

    void Parser::setStartPoint(double latitude, double longitude) {

        coordinatesConverter->setOrigin(latitude, longitude);
    }

    void Parser::setStartPoint(OSM_NODE node) {

        coordinatesConverter->setOrigin(node.latitude, node.longitude);
    }

    void Parser::setStartPoint(int id) {

        OSM_NODE node = this->getNodeByID(id);
        coordinatesConverter->setOrigin(node.latitude, node.longitude);
    }

    //set random start pose
   void Parser::setRandomStartPoint(){

       int id =  (int) (((double)rand() / RAND_MAX) * size_of_nodes);
       coordinatesConverter->setOrigin(nodes[id].latitude, nodes[id].longitude);
   }

   void Parser::setNewMap(std::string xml) {

        this->xml = xml;
        //parse();
   }

    void Parser::setTypeOfWays(std::vector<std::string> types){

        this->types_of_ways = types;
    }

   void Parser::setInterpolationMaxDistance(double param) {
        this->interpolation_max_distance = param;
   }


//private functions

   void Parser::createMarkers() {

        position_marker.header.frame_id = map_frame;
        position_marker.header.stamp =  ros::Time::now();
        position_marker.ns =  "work_space";
        position_marker.action = visualization_msgs::Marker::ADD;
        position_marker.pose.orientation.w = 1.0;

        position_marker.id = 0;

        position_marker.type = visualization_msgs::Marker::CYLINDER;

        position_marker.scale.z = 0.05;

        position_marker.lifetime = ros::Duration(100);

        //yellow marker
        position_marker.color.r = 1.0f;
        position_marker.color.g = 1.0f;
        position_marker.color.b = 0.0f;
        position_marker.color.a = 0.5;

        target_marker = position_marker;
        target_marker.type = visualization_msgs::Marker::ARROW;
        //red marker
        target_marker.color.g = 0.0f;
   }

//parse all ways in osm maps if osm_key = "null"
//else parse selected type of way.
//example:
// osm_key = "highway"
// osm_value = "footway"
// will selected only footways
   void Parser::createWays(TiXmlHandle *hRootWay, TiXmlHandle *hRootNode, std::vector<std::string> osm_value, bool onlyFirstElement) {

        //ADDED for interpolation
        //------------------------------------------
        //getting all OSM nodes for calculating distance between two nodes on the route
        std::vector<OSM_NODE_WITH_ID> nodes;
        OSM_NODE_WITH_ID nodeTmp;
        TiXmlElement *nodeElement = hRootNode->Element();

        for (nodeElement; nodeElement; nodeElement = nodeElement->NextSiblingElement("node")) {

            nodeElement->Attribute("id", &nodeTmp.id);
            nodeElement->Attribute("lat", &nodeTmp.node.latitude);
            nodeElement->Attribute("lon", &nodeTmp.node.longitude);
            nodes.push_back(nodeTmp);
        }
        //------------------------------------------

        ways.clear();
        table.clear();
        size_of_nodes = 0;

        TiXmlElement *wayElement = hRootWay->Element();

        OSM_WAY wayTmp;
        TiXmlElement *tag;

        //prejde vsetky elementy way
        for (wayElement; wayElement; wayElement = wayElement->NextSiblingElement("way")) {

            tag = wayElement->FirstChildElement("tag");
            wayElement->Attribute("id", &wayTmp.id);

            //prejde vsetky elementy tag
            while (tag != NULL) {

                if (isSelectedWay(tag, osm_value)) {

                    wayElement->Attribute("id", &wayTmp.id);
                    getNodesInWay(wayElement, &wayTmp, nodes); //finding all nodes located in selected way
                    ways.push_back(wayTmp);
                    if (onlyFirstElement) return;
                    break;
                }
                tag = tag->NextSiblingElement("tag");
            }
        }
   }

   bool Parser::isSelectedWay(TiXmlElement *tag, std::vector<std::string> values) {

        std::string key(tag->Attribute("k"));
        std::string value(tag->Attribute("v"));

        if (values.size() == 0)
            return true; //selected all

        if (key == "highway") {
            if (values[0] == "all")
                return true; //selected all ways with key highway

            for (int i = 0; i < values.size(); i++) {
                if (value == values[i])
                    return true;
            }
        }
        return false;
   }

//finding nodes located on way and fill way.nodesId
   void Parser::getNodesInWay(TiXmlElement *wayElement, OSM_WAY *way, std::vector<OSM_NODE_WITH_ID> nodes) {

        TiXmlHandle hRootNode(0);
        TiXmlElement *nodeElement;
        int id;

        way->nodesId.clear();

        nodeElement = wayElement->FirstChild("nd")->ToElement();
        hRootNode = TiXmlHandle(nodeElement);

        nodeElement = hRootNode.Element();

        TRANSLATE_TABLE tableTmp;

        //ADDED for interpolation
        //------------------------------------------
        OSM_NODE_WITH_ID node_new;              //Between this nodes will calculate
        OSM_NODE_WITH_ID node_old;             //interpolation
        std::vector<OSM_NODE> new_nodes_list; //List of interpolated nodes
        int counter = 0;
        //------------------------------------------

        for (nodeElement; nodeElement; nodeElement = nodeElement->NextSiblingElement("nd")) {

            nodeElement->Attribute("ref", &id);

            //ADDED for interpolation
            //------------------------------------------
            if (counter > 0) {
                memcpy(&node_old, &node_new, sizeof(node_old));     //save information about node
                node_new = getNodeByOsmId(nodes, id);               //get information about new node

                new_nodes_list = getInterpolatedNodes(node_old.node, node_new.node); //do interpolation
                tableTmp.oldID = -1; //interpolated node hasn't any ID in xml

                for (int i = 0; i < new_nodes_list.size(); i++) { //get the interpolated nodes

                    memcpy(&node_old.node, &new_nodes_list[i], sizeof(OSM_NODE)); //copy information about lon and lat
                    node_old.id = size_of_nodes;           //set the ID
                    tableTmp.newID = size_of_nodes++;      //set the ID in translate table and increment ID
                    table.push_back(tableTmp);
                    way->nodesId.push_back(tableTmp.newID); //add interpolated node on way
                    interpolated_nodes.push_back(node_old); //add interpolated node in buffer. It will use later.
                }

            } else {
                node_new = getNodeByOsmId(nodes, id);       //get information about first node in way
            }
            //------------------------------------------

            tableTmp.oldID = id;
            int ret;
            if (!translateID(tableTmp.oldID, &ret)) {

                tableTmp.newID = size_of_nodes++;
                table.push_back(tableTmp);
                way->nodesId.push_back(tableTmp.newID);

            } else {
                way->nodesId.push_back(ret);
            }
            //ADDED for interpolation
            //------------------------------------------
            counter++;
            //------------------------------------------

        }

   }

//ADDED for interpolation
//------------------------------------------
//Finding nodes by OSM ID
   Parser::OSM_NODE_WITH_ID Parser::getNodeByOsmId(std::vector<OSM_NODE_WITH_ID> nodes, int id) {

        for (int i = 0; i < nodes.size(); i++) {
            if (nodes[i].id == id)
                return nodes[i];
        }
        ROS_ERROR("OSM planner: nenaslo ziadnu nodu - toto by sa nemalo stat");
        return nodes[0];
   }

//INTERPOLATION - main algorithm
   std::vector<Parser::OSM_NODE> Parser::getInterpolatedNodes(OSM_NODE node1, OSM_NODE node2) {

        std::vector<OSM_NODE> new_nodes;
        OSM_NODE new_node;

        double dist = coordinatesConverter->getDistance(node1, node2);

        int count_new_nodes = dist / interpolation_max_distance;    //calculate number of new interpolated nodes

        for (int i = 0; i < count_new_nodes; i++) {
            //For example: if count_new_nodes = 2
            //1. latitude = (2 * node1.latitude - 1*node2.latitude)/3
            //2. latitude = (1 * node1.latitude - 2*node2.latitude)/3
            new_node.latitude =
                    ((count_new_nodes - i) * node1.latitude + (i + 1) * node2.latitude) / (count_new_nodes + 1);
            new_node.longitude =
                    ((count_new_nodes - i) * node1.longitude + (i + 1) * node2.longitude) / (count_new_nodes + 1);
            new_nodes.push_back(new_node);
        }
        return new_nodes;
   }
//------------------------------------------


   //parse all osm nodes and select nodes located in ways (footways)
   void Parser::createNodes(TiXmlHandle *hRootNode, bool onlyFirstElement) {

        nodes.clear();
        nodes.resize(table.size());

        TiXmlElement *nodeElement = hRootNode->Element();

        int id;
        for (nodeElement; nodeElement; nodeElement = nodeElement->NextSiblingElement("node")) {

            nodeElement->Attribute("id", &id);
            int ret;
            if (!translateID(id, &ret)) {
                continue;
            }

            nodeElement->Attribute("lat", &nodes[ret].latitude);
            nodeElement->Attribute("lon", &nodes[ret].longitude);
            if (onlyFirstElement) return;
        }

        //ADDED for interpolation
//------------------------------------------
        //Copy interpolated nodes to all nodes
        //interpolated_nodes[i].node - lat and lon information
        //interpolated_nodes[i].id - index of nodes from traslate table
        for (int i = 0; i < interpolated_nodes.size(); i++) {

            memcpy(&nodes[interpolated_nodes[i].id], &interpolated_nodes[i].node, sizeof(OSM_NODE));
        }
        //------------------------------------------

   }

   //creating graph for dijkstra algorithm
   void Parser::createNetwork() {

        networkArray.clear();
        networkArray.resize(nodes.size(), std::vector<float>(nodes.size(), 0.0));

        double distance = 0;
        //prejde vsetky cesty
        for (int i = 0; i < ways.size(); i++) {

            //prejde vsetky uzly na ceste
            for (int j = 0; j < ways[i].nodesId.size() - 1; j++) {

                //vypocita vzdialenost medzi susednimi uzlami
                distance = coordinatesConverter->getDistance(nodes[ways[i].nodesId[j]], nodes[ways[i].nodesId[j + 1]]);
              //  ROS_INFO("[%d %d]",ways[i].nodesId[j], ways[i].nodesId[j + 1]);
              //  ROS_INFO("dist %f", distance);

                if (networkArray[ways[i].nodesId[j]][ways[i].nodesId[j + 1]] != 0)
                    ROS_ERROR("OSM planner: pozicia [%d, %d] je obsadena - toto by nemalo nastat", ways[i].nodesId[j],
                              ways[i].nodesId[j + 1]);

                networkArray[ways[i].nodesId[j]][ways[i].nodesId[j + 1]] = (float) distance;
                networkArray[ways[i].nodesId[j + 1]][ways[i].nodesId[j]] = (float) distance;

            }

        }
        interpolated_nodes.clear();
        table.clear();

       /* networkArray.clear();
        for (int i = 0; i < networkArray.size(); i++) {
            networkArray[i].clear();
        }*/

   }


//preklada stare osm node ID na nove osm node ID (cielom bolo vytvorit usporiadane indexovanie)
   bool Parser::translateID(int id, int *ret_value) {

        for (int i = 0; i < table.size(); i++) {
            if (table[i].oldID == id) {
                ret_value[0] = table[i].newID;
                return true;
            }
        }
        return false;
   }

}


