//
// Created by michal on 31.12.2016.
//


#include <osm_planner/osm_parser.h>

OsmParser::OsmParser(std::string xml){

        ros::NodeHandle n;

        std::string topic_name;

        //get the parameters
        n.param<std::string>("map_frame", map_frame, "/map");
        n.param<std::string>("topic_shortest_path", topic_name, "/shortest_path" );
        n.param<bool>("/visualization", visualization, false);

        //publishers
        shortest_path_pub = n.advertise<nav_msgs::Path>(topic_name, 10);

        if (visualization) {
            //Publishers for visualization
            position_marker_pub = n.advertise<visualization_msgs::Marker>("/position_marker", 5);
            target_marker_pub = n.advertise<visualization_msgs::Marker>("/target_marker", 1);

            path_pub = n.advertise<nav_msgs::Path>("/route_network", 10);
            refused_path_pub = n.advertise<nav_msgs::Path>("/refused_path", 10);
        } else ROS_ERROR("visualization false");
        TiXmlDocument doc(xml);
        TiXmlNode* osm;
        TiXmlNode* node;
        TiXmlNode* way;

        TiXmlHandle hRootNode(0);
        TiXmlHandle hRootWay(0);
        TiXmlHandle hRootTag(0);

        bool loadOkay = doc.LoadFile();
        if (loadOkay)
        {
            ROS_INFO("loaded map: %s", xml.c_str());
        }
        else
        {
            ROS_ERROR("Failed to load file %s", xml.c_str());
            throw std::runtime_error("Failed to load xml");
        }

        osm = doc.FirstChildElement();
        node = osm->FirstChild("node");
        way = osm->FirstChild("way");
        TiXmlElement* nodeElement = node->ToElement();
        TiXmlElement* wayElement = way->ToElement();

        hRootNode=TiXmlHandle(nodeElement);
        hRootWay=TiXmlHandle(wayElement);

        createWays(&hRootWay,"highway", "footway");
        createNodes(&hRootNode);
        createNetwork();
        createMarkers();
    }


        /**PUBLISHING FUNCTIONS**/

void OsmParser::publishPoint(double latitude, double longitude, int marker_type){

    if (!visualization)
        return;

    geometry_msgs::Point point;
    point.z = 0;
    point.x = 0;
    point.y = 0;

    point.x = Haversine::getCoordinateX(startPoint.longitude, longitude, startPoint.latitude, latitude);
    point.y = Haversine::getCoordinateY(startPoint.latitude, latitude);

    switch (marker_type){
         case CURRENT_POSITION_MARKER:
            position_marker.points.clear();
            position_marker.points.push_back(point);
            position_marker_pub.publish(position_marker);
            break;
         case TARGET_POSITION_MARKER:
            target_marker.points.clear();
            target_marker.points.push_back(point);
            target_marker_pub.publish(target_marker);
            break;
     }
}


void OsmParser::publishPoint(int pointID, int marker_type) {

    if (!visualization)
        return;

    geometry_msgs::Point point;
    point.z = 0;
    point.x = 0;
    point.y = 0;

    point.x = Haversine::getCoordinateX(startPoint, nodes[pointID]);
    point.y = Haversine::getCoordinateY(startPoint, nodes[pointID]);

    switch (marker_type){
        case CURRENT_POSITION_MARKER:
            position_marker.points.clear();
            position_marker.points.push_back(point);
            position_marker_pub.publish(position_marker);
            break;
        case TARGET_POSITION_MARKER:
            target_marker.points.clear();
            target_marker.points.push_back(point);
            target_marker_pub.publish(target_marker);
            break;
    }

}

//publishing all paths
    void OsmParser::publishRouteNetwork(){

    if (!visualization)
        return;

    OSM_NODE testNode;
    nav_msgs::Path path;
    path.header.frame_id = map_frame;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    for (int i = 0; i < ways.size(); i++){
        path.poses.clear();

        for (int j = 0; j < ways[i].nodesId.size(); j++){

            pose.pose.position.x = Haversine::getCoordinateX(startPoint, nodes[ways[i].nodesId[j]]);
            pose.pose.position.y = Haversine::getCoordinateY(startPoint, nodes[ways[i].nodesId[j]]);
            path.poses.push_back(pose);

        }
        usleep(100000);

        path.header.stamp = ros::Time::now();
        path_pub.publish(path);
    }
}

//publishing defined path
void OsmParser::publishRefusedPath(std::vector<int> nodesInPath) {

    if (!visualization)
        return;

    nav_msgs::Path refused_path;
    refused_path.poses.clear();
    refused_path.header.frame_id = map_frame;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    for (int i = 0; i < nodesInPath.size(); i++) {

        pose.pose.position.x = Haversine::getCoordinateX(startPoint, nodes[nodesInPath[i]]);
        pose.pose.position.y = Haversine::getCoordinateY(startPoint, nodes[nodesInPath[i]]);
        refused_path.poses.push_back(pose);
    }

    refused_path.header.stamp = ros::Time::now();
    refused_path_pub.publish(refused_path);
}

//publishing defined path with target geographic point
void OsmParser::publishPath(std::vector<int> nodesInPath, double target_lat, double target_lon) {

    sh_path.poses.clear();
    sh_path.header.frame_id = map_frame;

    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    for (int i = 0; i < nodesInPath.size(); i++) {

        ROS_WARN("shortest path: node id %d", nodesInPath[i]);

        pose.pose.position.x = Haversine::getCoordinateX(startPoint, nodes[nodesInPath[i]]);
        pose.pose.position.y = Haversine::getCoordinateY(startPoint, nodes[nodesInPath[i]]);
        pose.header.seq = i;
        sh_path.poses.push_back(pose);
    }

    pose.pose.position.x = Haversine::getCoordinateX(startPoint.longitude, target_lon, startPoint.latitude, target_lat);
    pose.pose.position.y = Haversine::getCoordinateY(startPoint.latitude, target_lat);
    pose.header.seq = pose.header.seq + 1;
    sh_path.poses.push_back(pose);
    sh_path.header.stamp = ros::Time::now();
    shortest_path_pub.publish(sh_path);

}

//todo spravit aj obnovu bodov
void OsmParser::deleteEdgeOnGraph(int nodeID_1, int nodeID_2){

    networkArray[nodeID_1][nodeID_2] = 0;
    networkArray[nodeID_2][nodeID_1] = 0;

}

        /* GETTERS */

//todo - mozno by bolo lepsie, aby tato funkcia robila funkciu createNerwork() a pole by bolo globalne len v classe Dijkstra
//getter for dijkstra algorithm
std::vector< std::vector<double> > OsmParser::getGraphOfVertex(){
    return networkArray;
}

int OsmParser::getNearestPoint(double lat, double lon){

    OSM_NODE point;
    point.longitude = lon;
    point.latitude = lat;
    point.id = 0;

    double distance = Haversine::getDistance(point, nodes[0]);
    double minDistance = distance;


    for (int i = 0; i < nodes.size(); i++){
       distance = Haversine::getDistance(point, nodes[i]);

        if (minDistance > distance){
           minDistance = distance;
           point.id = nodes[i].id;
        }
    }
    return point.id;
}

//return OSM NODE, which contains geographics coordinates
OsmParser::OSM_NODE OsmParser::getNodeByID(int id){

    return nodes[id];
}
        /* SETTERS */

void OsmParser::setStartPoint(double latitude, double longitude){

    this->startPoint.id = 0;
    this->startPoint.latitude = latitude;
    this->startPoint.longitude = longitude;
}


//private functions

void OsmParser::createMarkers(){

    if (!visualization)
        return;

    visualization_msgs::Marker line_strip, line_list;

    position_marker.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = map_frame;
    position_marker.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    position_marker.ns = line_strip.ns = line_list.ns = "work_space";
    position_marker.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    position_marker.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    position_marker.id = 0;
    line_list.id = 2;

    position_marker.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::ARROW;

    line_list.scale.x = 0.05;
    line_list.scale.y = 0.05;
    line_list.scale.z = 0.05;

    line_list.color.r = 1.0f;
    line_list.color.g = 0.0f;
    line_list.color.b = 0.0f;
    line_list.color.a = 1.0;

    line_list.lifetime = ros::Duration(100);

    position_marker.scale.x = 0.05;
    position_marker.scale.y = 0.05;
    position_marker.scale.z = 0.05;

    position_marker.lifetime = ros::Duration(100);

    //yellow marker
    position_marker.color.r = 1.0f;
    position_marker.color.g = 1.0f;
    position_marker.color.b = 0.0f;
    position_marker.color.a = 1.0;

    target_marker = position_marker;
    //red marker
    target_marker.color.g = 0.0f;

}

//parse all ways in osm maps if osm_key = "null"
//else parse selected type of way.
//example:
// osm_key = "highway"
// osm_value = "footway"
// will selected only footways
    void OsmParser::createWays(TiXmlHandle* hRootWay, std::string osm_key, std::string osm_value){

        ways.clear();
        TiXmlElement* wayElement = hRootWay->Element();

        OSM_WAY wayTmp;
        TiXmlElement* tag;

        //prejde vsetky elementy way
        for( wayElement; wayElement; wayElement = wayElement->NextSiblingElement("way")){

            tag = wayElement->FirstChildElement("tag");
            wayElement->Attribute("id", &wayTmp.id);

            //prejde vsetky elementy tag
            while (tag != NULL){

                std::string key(tag->Attribute("k"));
                std::string value(tag->Attribute("v"));

                if (key == "null" || (key == osm_key && value == osm_value)){

                    wayElement->Attribute("id", &wayTmp.id);
                    getNodesInWay(wayElement, &wayTmp); //finding all nodes located in selected way
                    ways.push_back(wayTmp);
                    break;
                }
                tag = tag->NextSiblingElement("tag");
            }
        }
    }

    //parse all osm nodes and select nodes located in ways (footways)
    void OsmParser::createNodes(TiXmlHandle *hRootNode){

        nodes.clear();
        nodes.resize(table.size());

        TiXmlElement *nodeElement = hRootNode->Element();

        int id;
        int i = 0;

        for( nodeElement; nodeElement; nodeElement = nodeElement->NextSiblingElement("node")){


            nodeElement->Attribute("id", &id);

            int ret;
            if (!translateID(id, &ret)){
                continue;
            }

            nodes[ret].id = ret;
            nodeElement->Attribute("lat", &nodes[ret].latitude);
            nodeElement->Attribute("lon", &nodes[ret].longitude);
        }

    }

    //creating graph for dijkstra algorithm
    void OsmParser::createNetwork(){

        networkArray.resize(nodes.size());
        //ROS_ERROR("size %d ", networkArray.size());
        for (int i = 0; i < networkArray.size(); i++){
            networkArray[i].resize(nodes.size());
            //  ROS_WARN("size %d ", networkArray[i].size());
        }

        double distance = 0;
        //prejde vsetky cesty
        for (int i = 0; i < ways.size(); i++){

            //prejde vsetky uzly na ceste
            for (int j = 0; j < ways[i].nodesId.size() - 1; j++){
                //vypocita vzdialenost medzi susednimi uzlami
                distance = Haversine::getDistance(nodes[ways[i].nodesId[j]], nodes[ways[i].nodesId[j + 1]]);
                //
                if (networkArray[ways[i].nodesId[j]] [ways[i].nodesId[j + 1]] != 0)
                    ROS_ERROR("pozicia [%d, %d] je obsadena - toto by nemalo nastat",ways[i].nodesId[j], ways[i].nodesId[j + 1]);
                networkArray[ways[i].nodesId[j]] [ways[i].nodesId[j + 1]] = distance;
                networkArray[ways[i].nodesId[j + 1]] [ways[i].nodesId[j]] = distance;

            }

        }

    }

    //finding nodes located on way and fill way.nodesId
    void OsmParser::getNodesInWay(TiXmlElement* wayElement, OSM_WAY *way){

        TiXmlHandle hRootNode(0);
        TiXmlHandle hRootNd(0);
        TiXmlElement* nodeElement;
        int id;
        static int id_new = 0;

        way->nodesId.clear();

        nodeElement = wayElement->FirstChild("nd")->ToElement();
        hRootNode = TiXmlHandle(nodeElement);

        nodeElement = hRootNode.Element();
        TRANSLATE_TABLE tableTmp;

        for( nodeElement; nodeElement; nodeElement = nodeElement->NextSiblingElement("nd")) {

            nodeElement->Attribute("ref", &id);
            tableTmp.oldID = id;
            int ret;
            if (!translateID(tableTmp.oldID, &ret)){

                tableTmp.newID = id_new++;
                table.push_back(tableTmp);
                way->nodesId.push_back(tableTmp.newID);

            } else {
                way->nodesId.push_back(ret);
            }
        }

    }

//preklada stare osm node ID na nove osm node ID (cielom bolo vytvorit usporiadane indexovanie)
    bool OsmParser::translateID(int id, int *ret_value){

        for (int i = 0; i < table.size(); i++){
            if (table[i].oldID == id) {
                ret_value[0] = table[i].newID;
                return true;
            }
        }
        return false;
    }



//Embedded class for calculating distance and bearing
//Functions was inspired by: http://www.movable-type.co.uk/scripts/latlong.html
double OsmParser::Haversine::getDistance(OsmParser::OSM_NODE node1, OsmParser::OSM_NODE node2){

   /*  Haversine formula:
    *  a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
    *  c = 2 ⋅ atan2( √a, √(1−a) )
    *  d = R ⋅ c
    *
    *  φ - latitude;
    *  λ - longitude;
    */

    double dLat = node2.latitude * DEG2RAD - node1.latitude * DEG2RAD;
    double dLon = node2.longitude * DEG2RAD - node1.longitude * DEG2RAD;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(node1.latitude * DEG2RAD) * cos(node2.latitude * DEG2RAD) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return  R * c ;
}

double OsmParser::Haversine::getCoordinateX(double lon1, double lon2, double lat1, double lat2){

    double dLon = lon2 * DEG2RAD - lon1 * DEG2RAD;
    double latAverage = (lat1 + lat2)/2;
    double a = cos(latAverage * DEG2RAD) * cos(latAverage * DEG2RAD) *
               sin(dLon/2) * sin(dLon/2);
    double dist = R * 2 * atan2(sqrt(a), sqrt(1-a));

    return lon1 < lon2 ? dist : -dist;
}

double OsmParser::Haversine::getCoordinateX(OsmParser::OSM_NODE node1, OsmParser::OSM_NODE node2){

    double dLon = node2.longitude * DEG2RAD - node1.longitude * DEG2RAD;
    double latAverage = (node1.latitude + node2.latitude)/2;
    double a = cos(latAverage * DEG2RAD) * cos(latAverage * DEG2RAD) *
               sin(dLon/2) * sin(dLon/2);
    double dist = R * 2 * atan2(sqrt(a), sqrt(1-a));

    return node1.longitude < node2.longitude ? dist : -dist;
}


double OsmParser::Haversine::getCoordinateY(double lat1, double lat2){

    static double R = 6371e3;
    double dLat = lat2 * DEG2RAD - lat1 * DEG2RAD;
    double a = sin(dLat/2) * sin(dLat/2);
    double dist = R * 2 * atan2(sqrt(a), sqrt(1-a));

    return lat1 < lat2 ? dist : -dist;
}

double OsmParser::Haversine::getCoordinateY(OsmParser::OSM_NODE node1, OsmParser::OSM_NODE node2){

    static double R = 6371e3;
    double dLat = node2.latitude * DEG2RAD - node1.latitude * DEG2RAD;
    double a = sin(dLat/2) * sin(dLat/2);
    double dist = R * 2 * atan2(sqrt(a), sqrt(1-a));

    return node1.latitude < node2.latitude ? dist : -dist;
}

double OsmParser::Haversine::getBearing(OsmParser::OSM_NODE node1, OsmParser::OSM_NODE node2) {

   /*   Haversine formula:
    *   a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
    *   c = 2 ⋅ atan2( √a, √(1−a) )
    *   d = R ⋅ c
    *
    *  φ - latitude;
    *  λ - longitude;
    */

    double dLon = node2.longitude * DEG2RAD - node1.longitude * DEG2RAD;

    double y = sin(dLon) * cos(node2.latitude * DEG2RAD);
    double x = cos(node1.latitude * DEG2RAD)*sin(node2.latitude * DEG2RAD) -
            sin(node1.latitude * DEG2RAD)*cos(node2.latitude * DEG2RAD)*cos(dLon);
    return atan2(y, x) * RAD2DEG;
}
