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


std::vector<geometry_msgs::Pose> poses;
nav_msgs::Odometry odom;

bool initialized_path;

//getting path
void pathCallback(const nav_msgs::Path::ConstPtr& msg) {

    ROS_WARN("SIMULATED ROBOT: Getting trajectory");
    sleep(1);
    poses.clear();

    for (int i = 0; i < msg->poses.size(); i++) {
        poses.push_back(msg->poses[i].pose);
        ROS_INFO("%d. Point: x = %f y = %f", i, poses[i].position.x, poses[i].position.y);
    }

    initialized_path = true;

}

int main(int argc, char **argv) {


	ros::init(argc, argv, "test_osm");
	ros::NodeHandle n;

    ros::Publisher position_pub = n.advertise<nav_msgs::Odometry>("position", 5);
    ros::Subscriber path_sub = n.subscribe("shortest_path", 1, &pathCallback);

    ros::ServiceClient cancel_point = n.serviceClient<osm_planner::cancelledPoint>("cancel_point");

    ros::ServiceClient set_source = n.serviceClient<osm_planner::newTarget>("init");
    ros::ServiceClient set_target = n.serviceClient<osm_planner::newTarget>("replanning");

    osm_planner::newTarget gps_srv;

    //wait all nodes was initialized
    sleep(3);

    //start gps position
    ROS_WARN("SIMULATED ROBOT: setting gps source");
    gps_srv.request.target.latitude = 48.1463634;
    gps_srv.request.target.longitude = 17.0734773;
    set_source.call(gps_srv);

//    sleep(3);
    ROS_WARN("SIMULATED ROBOT: setting gps target");
    //target gps position
    gps_srv.request.target.latitude = 48.1455653;
    gps_srv.request.target.longitude = 17.0728155;
    set_target.call(gps_srv);

    initialized_path = false;

    //simulated odometry
    odom.header.frame_id = "/base_link";
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    position_pub.publish(odom);

    int seq = 0;

    bool obstacle = false;

    ros::Rate rate(1);

    while(ros::ok()){

      if (initialized_path){

          if (seq > poses.size()){

              ROS_WARN("SIMULATED ROBOT: Target position was reached");
              initialized_path = false;
              continue;
          }


          //simulated robot moving
          odom.header.stamp = ros::Time::now();
          odom.pose.pose.position.x = poses[seq].position.x;
          odom.pose.pose.position.y = poses[seq].position.y;
          position_pub.publish(odom);


          // simulated obstacle in 10. point
          //trajectory must be replane
          if (seq == 10 && !obstacle){

              ROS_ERROR("SIMULATED ROBOT: Obstacle is detected. Trajectory must be replane");
              sleep(2);
              osm_planner::cancelledPoint point;
              point.request.pointID = seq;
              cancel_point.call(point);
              initialized_path = false;
              seq = 0;
              obstacle = true;
              continue;
          }

          seq++;
      }

        ros::spinOnce();
        rate.sleep();
    }


return 0;
}