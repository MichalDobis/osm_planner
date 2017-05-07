/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

#include <ros/ros.h>

#include <osm_planner/newTarget.h>
#include <osm_planner/cancelledPoint.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

std::vector<geometry_msgs::Pose> poses;
nav_msgs::Odometry odom;

bool initialized_path;
ros::Publisher position_pub;
ros::ServiceClient cancel_point;
ros::ServiceClient set_source;

ros::Timer timer;

double angle;

//getting path
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    static ros::Time start_time = ros::Time::now();
    double dt = (ros::Time::now() - start_time).toSec();
    start_time = ros::Time::now();

    //dt *=2;
    angle += dt * msg->angular.z;

    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    odom.pose.pose.position.x += msg->linear.x * dt * sin(angle);
    odom.pose.pose.position.y += msg->linear.x * dt * cos(angle);
    position_pub.publish(odom);

}


void updateCallback(const ros::TimerEvent&)
{

    timer.stop();
    osm_planner::newTarget gps_srv;
    ros::NodeHandle n;

    ROS_WARN("SIMULATED ROBOT: setting gps source");
    //start gps position
    gps_srv.request.latitude = 48.1532533367;
    gps_srv.request.longitude = 17.74406095;
    gps_srv.request.bearing = 0;
    //get from param if exists
    n.getParam("/source_lon", gps_srv.request.longitude);
    n.getParam("/source_lat",  gps_srv.request.latitude);

    //set_source.call(gps_srv);
   // sleep(5);
   // set_source.call(gps_srv);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "test_osm");
    ros::NodeHandle n;

    position_pub = n.advertise<nav_msgs::Odometry>("position", 5);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1, &cmdVelCallback);

    cancel_point = n.serviceClient<osm_planner::cancelledPoint>("cancel_point");
    set_source = n.serviceClient<osm_planner::newTarget>("init");

    timer = n.createTimer(ros::Duration(3), updateCallback);


    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;


    //simulated odometry
    odom.header.frame_id = "/base_link";
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    angle = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    position_pub.publish(odom);

    initialized_path = false;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate rate(20);

    while(ros::ok()){

        transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0) );
        tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_link"));

        rate.sleep();
    }


    return 0;
}