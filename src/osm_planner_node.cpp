/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

#include <osm_planner/osm_planner.h>
#include <nav_msgs/Odometry.h>

class OsmPlannerNode: osm_planner::Planner{
public:

    const static int GEOGRAPHICS_COORDINATES = 1;
    const static int CARTEZIAN_COORDINATES = 2;

    OsmPlannerNode(std::string file) : osm_planner::Planner(){

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
    }

private:

    /* Subscribers */
    ros::Subscriber position_sub;

    /* Services */
    ros::ServiceServer planning_service;


    bool planningCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res) {

        res.result = makePlan(req.target.latitude, req.target.longitude);
        return true;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

       setPositionFromGPS(msg->latitude, msg->longitude);
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

      setPositionFromOdom(msg->pose.pose.position);
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
