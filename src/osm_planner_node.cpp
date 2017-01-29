/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

#include <ros/ros.h>
#include <osm_planner/dijkstra.h>
#include <osm_planner/osm_parser.h>
#include <osm_planner/newTarget.h>
#include <osm_planner/cancelledPoint.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>

class OsmPlanner{
public:

    const static int GEOGRAPHICS_COORDINATES = 1;
    const static int CARTEZIAN_COORDINATES = 2;

    OsmPlanner(std::string file) :
            osm(file),
            dijkstra(osm.getGraphOfVertex()), targetID(100), sourceID(130), initialized(false)
    {

        //init ros topics and services
        ros::NodeHandle n;

        std::string topic_name;
        int topic_type;
        n.param<std::string>("topic_position_name", topic_name, "/position");
        n.param<int>("topic_position_type", topic_type, 1);

        //subscribers
        if (topic_type == GEOGRAPHICS_COORDINATES){
            position_sub = n.subscribe(topic_name, 1, &OsmPlanner::gpsCallback, this);
        } else if (topic_type == CARTEZIAN_COORDINATES) {
            position_sub = n.subscribe(topic_name, 1, &OsmPlanner::odometryCallback, this);
        } else throw std::runtime_error("Load bad parameter topic_position_type");

        //services
        replanning_service = n.advertiseService("replanning", &OsmPlanner::replanning, this);
        cancel_point_service = n.advertiseService("cancel_point", &OsmPlanner::cancelPoint, this);
        init_service = n.advertiseService("init", &OsmPlanner::init, this);


        //****************TEST PLANNING PATH FROM PARAM**********************//
        //-------------------------------------------------------------------//


        bool test;
        n.param<bool>("/test", test, false);
        if (test) {

            //subscribers for test
            replanning_sub = n.subscribe("replanning", 1, &OsmPlanner::changeTarget, this);
            change_source_sub = n.subscribe("change_source", 1, &OsmPlanner::changeSource, this);

            //finding nereast OSM node
            double source_lat, source_lon;
            n.param<double>("target_lon", target_longitude, 1);
            n.param<double>("target_lat", target_latitude, 1);
            n.param<double>("source_lon", source_lon, 1);
            n.param<double>("source_lat", source_lat, 1);

            sourceID = osm.getNearestPoint(source_lat, source_lon);
            ROS_INFO("source ID %d", sourceID);
           osm.setStartPoint(source_lat, source_lon);
            sleep(1);
            //osm.publishPoint(sourceID, OsmParser::CURRENT_POSITION_MARKER);
            osm.publishPoint(source_lat, source_lon, OsmParser::CURRENT_POSITION_MARKER);

            //draw route network
           osm.publishRouteNetwork();
            sleep(1);

            targetID = osm.getNearestPoint(target_latitude, target_longitude);

            ROS_INFO("target ID %d", targetID);
            //sleep(1);
            osm.publishPoint(target_latitude, target_longitude, OsmParser::TARGET_POSITION_MARKER);

            //planning and publish final path
            osm.publishPath(dijkstra.findShortestPath(sourceID, targetID), target_latitude, target_longitude);

            initialized = true;
        }
        //-------------------------------------------------------------------//
    }
private:

    OsmParser osm;
    Dijkstra dijkstra;

    int sourceID;
    int targetID;

    bool initialized;

    double target_longitude;
    double target_latitude;

    /* Subscribers */
    ros::Subscriber replanning_sub;
    ros::Subscriber position_sub;
    ros::Subscriber change_source_sub;

    /* Services */
    ros::ServiceServer replanning_service;
    ros::ServiceServer init_service;
    ros::ServiceServer cancel_point_service;

    bool init(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res){

        initMap(req.target.latitude, req.target.longitude);
        return true;
    }

    bool replanning(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res){

        target_latitude = req.target.latitude;
        target_longitude = req.target.longitude;

        targetID = osm.getNearestPoint(target_latitude, target_longitude);
        osm.publishPath(dijkstra.findShortestPath(sourceID, targetID), target_latitude, target_longitude);

        //todo dorobit v pripade, ze sa nenaplanuje, tak res.success = false
        res.success = true;
        ROS_WARN("getting target ID %d planning trajectory...", targetID);

        return true;
    }

    bool cancelPoint(osm_planner::cancelledPoint::Request &req, osm_planner::cancelledPoint::Response &res){

        //get current shortest path - vector of osm nodes IDs
       std::vector <int> path = dijkstra.getSolution(targetID);

        //if index is greather then array size
        if (req.pointID >= path.size()){
            res.success = false;
            return true;
        }

        //get the OSM nodes
        OsmParser::OSM_NODE node1 = osm.getNodeByID(path[req.pointID]);
        OsmParser::OSM_NODE node2 = osm.getNodeByID(path[req.pointID + 1]);

        //for drawing deleted path
        std::vector<int> refused_path(2);
        refused_path[0] = node1.id;
        refused_path[1] = node2.id;
        osm.publishRefusedPath(refused_path);

        //delete edge between two osm nodes
        osm.deleteEdgeOnGraph(node1.id, node2.id);

        //replanning shorest path
        sourceID = path[req.pointID];   //return back to last position
        dijkstra.setGraph(osm.getGraphOfVertex());
        osm.publishPath(dijkstra.findShortestPath(sourceID, targetID), target_latitude, target_longitude);
        //todo dorobit v pripade, ze sa nenaplanuje, tak res.success = false
        res.success = true;
        return true;
    }

    //*******************************TEST********************************//
    //-------------------------------------------------------------------//
    void changeTarget(const std_msgs::Int32::ConstPtr& msg) {

        targetID = msg->data;
        ROS_WARN("change target %d", targetID);

        osm.publishPath(dijkstra.getSolution(targetID), osm.getNodeByID(targetID).latitude, osm.getNodeByID(targetID).longitude);
        osm.publishPoint(targetID, OsmParser::TARGET_POSITION_MARKER);

    }

    //len pre test, bude sa pouzivat gpsCallback
    void changeSource(const std_msgs::Int32::ConstPtr& msg) {

        sourceID = msg->data;

        ROS_WARN("change source %d", sourceID);
        osm.publishPoint(sourceID, OsmParser::CURRENT_POSITION_MARKER);
        sleep(1);
        osm.publishPath(dijkstra.findShortestPath(sourceID, targetID), osm.getNodeByID(sourceID).latitude, osm.getNodeByID(sourceID).longitude);

    }
    //-------------------------------------------------------------------//


    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

        if (!initialized)
            return;

        double lat = msg->latitude;
        double lon = msg->longitude;

        osm.publishPoint(lat, lon, OsmParser::CURRENT_POSITION_MARKER);
        sourceID = osm.getNearestPoint(lat, lon);
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

        if (!initialized)
            return;

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        sourceID = osm.getNearestPointXY(x, y);
        ROS_INFO("change position ID %d", sourceID);

        osm.publishPoint(sourceID, OsmParser::CURRENT_POSITION_MARKER);
    }

    void initMap(double lat, double lon){

        osm.setStartPoint(lat, lon);
        sourceID = osm.getNearestPoint(lat,lon);
        osm.publishPoint(lat, lon, OsmParser::CURRENT_POSITION_MARKER);
        ROS_INFO("source ID %d", sourceID);

        //draw paths network
        osm.publishRouteNetwork();

        initialized = true;
    }

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_osm");
	ros::NodeHandle n;

	std::string file = "skuska.osm";
	n.getParam("filepath", file);

    OsmPlanner osm_planner(file);
    ros::spin();

return 0;
}
