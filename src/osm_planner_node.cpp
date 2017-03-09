/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

#include <osm_planner/osm_planner.h>
#include <nav_msgs/Odometry.h>

class OsmPlannerNode{
public:

    const static int GEOGRAPHICS_COORDINATES = 1;
    const static int CARTEZIAN_COORDINATES = 2;

    OsmPlannerNode(std::string file) : planner(){

        //init ros topics and services
        ros::NodeHandle n;

        std::string topic_name;
        int topic_type;
        n.param<std::string>("topic_position_name", topic_name, "/position");
        n.param<int>("topic_position_type", topic_type, 1);

        //subscribers
        if (topic_type == GEOGRAPHICS_COORDINATES){
            position_sub = n.subscribe(topic_name, 1, &OsmPlannerNode::gpsCallback, this);
        } else if (topic_type == CARTEZIAN_COORDINATES) {
            position_sub = n.subscribe(topic_name, 1, &OsmPlannerNode::odometryCallback, this);
        } else throw std::runtime_error("Load bad parameter topic_position_type");

        //services
        planning_service = n.advertiseService("planning", &OsmPlannerNode::planningCallback, this);
        cancel_point_service = n.advertiseService("cancel_point", &OsmPlannerNode::cancelPointCallback, this);

    }
private:

    osm_planner::Planner planner;

    /* Subscribers */
    ros::Subscriber position_sub;

    /* Services */
    ros::ServiceServer planning_service;
    ros::ServiceServer cancel_point_service;


    bool planningCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res) {

        nav_msgs::Path *path;
        res.result = planner.planning(req.target.latitude, req.target.longitude, path);
        return true;
    }

    bool cancelPointCallback(osm_planner::cancelledPoint::Request &req, osm_planner::cancelledPoint::Response &res){

       res.result = planner.cancelPoint(req.pointID);
        return true;
    }


    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

       planner.setPositionFromGPS(msg->latitude, msg->longitude);
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

      planner.setPositionFromOdom(msg->pose.pose.position);
    }

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_osm");
	ros::NodeHandle n;

	std::string file = "skuska.osm";
	n.getParam("filepath", file);

    OsmPlannerNode osm_planner(file);
    ros::spin();

return 0;
}
