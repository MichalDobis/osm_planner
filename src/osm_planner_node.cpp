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
            dijkstra(), targetID(100), sourceID(130), initialized(false)
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

        ROS_WARN("OSM planner: Waiting for init...");
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

        if (!initialized) {
            res.success = false;
            return true;
        }
        target_latitude = req.target.latitude;
        target_longitude = req.target.longitude;

        ros::Time start_time = ros::Time::now();
        targetID = osm.getNearestPoint(target_latitude, target_longitude);
        ROS_WARN("OSM planner: Planning trajectory...");
        osm.publishPath(dijkstra.findShortestPath(osm.getGraphOfVertex(), sourceID, targetID), target_latitude, target_longitude);
        ROS_INFO("OSM planner: Plan time %f ",(ros::Time::now() - start_time).toSec());

        //todo dorobit v pripade, ze sa nenaplanuje, tak res.success = false
        res.success = true;

        return true;
    }

    bool cancelPoint(osm_planner::cancelledPoint::Request &req, osm_planner::cancelledPoint::Response &res){

        if (!initialized) {
            res.success = false;
            return true;
        }

        //get current shortest path - vector of osm nodes IDs
       std::vector <int> path = dijkstra.getSolution();

        //if index is greather then array size
        if (req.pointID >= path.size()){
            res.success = false;
            return true;
        }

        //for drawing deleted path
        std::vector<int> refused_path(2);
        refused_path[0] = path[req.pointID];
        refused_path[1] = path[req.pointID + 1];
        osm.publishRefusedPath(refused_path);

        //delete edge between two osm nodes
        osm.deleteEdgeOnGraph(path[req.pointID], path[req.pointID + 1]);

        //replanning shorest path
        sourceID = path[req.pointID];   //return back to last position

        osm.publishPath(dijkstra.findShortestPath(osm.getGraphOfVertex(), sourceID, targetID), target_latitude, target_longitude);
        //todo dorobit v pripade, ze sa nenaplanuje, tak res.success = false
        res.success = true;
        return true;
    }


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

        osm.publishPoint(msg->pose.pose.position, OsmParser::CURRENT_POSITION_MARKER);
    }

    void initMap(double lat, double lon){

        osm.parse();
        osm.setStartPoint(lat, lon);
        sourceID = osm.getNearestPoint(lat,lon);
        osm.publishPoint(lat, lon, OsmParser::CURRENT_POSITION_MARKER);
        //draw paths network
        osm.publishRouteNetwork();
        initialized = true;
        ROS_INFO("OSM planner: Initialized. Waiting for request of plan...");
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
