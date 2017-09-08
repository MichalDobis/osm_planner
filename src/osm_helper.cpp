/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv) {

    ros::init(argc, argv, "osm_helper");
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(20);

    while(ros::ok()){

        transform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0,0,0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_link"));

        rate.sleep();
    }

    return 0;
}