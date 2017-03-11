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

    OsmPlannerNode() : osm_planner::Planner(){

        //init ros topics and services
        ros::NodeHandle n;

        std::string topic_name;
        int topic_type;
        n.param<std::string>("topic_position_name", topic_name, "/position");
        n.param<int>("topic_position_type", topic_type, 1);

        //subscribers
        gps_sub = n.subscribe(topic_name, 1, &OsmPlannerNode::gpsCallback, this);
        odom_sub = n.subscribe(topic_name, 1, &OsmPlannerNode::odometryCallback, this);

        //services
        plan_service = n.advertiseService("make_plan", &OsmPlannerNode::makePlanCallback, this);
    }

    void update(){
        updatePose();
    }

private:

    /* Subscribers */
    ros::Subscriber gps_sub;
    ros::Subscriber odom_sub;

    /* Services */
    ros::ServiceServer plan_service;


    bool makePlanCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res) {

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

    OsmPlannerNode osm_planner;

    ros::Rate rate(2);

    while (ros::ok()) {

        osm_planner.update();
        ros::spinOnce();
    }

return 0;
}
