//
// Created by michal on 31.12.2016.
// Modify and refactor by michal on 11.11.2018
//
#include <osm_planner/osm_localization.h>
#include <ros/ros.h>


namespace osm_planner {

    Localization::Localization(std::shared_ptr<osm_planner::Parser> map, std::string name) {

        this->map_ = map;
        this->name_ = name;

        // Set default values
        positions_.id = -1;
        positions_.cartesian_pose.position.x = 0;
        positions_.cartesian_pose.position.y = 0;
        positions_.is_set_from_gps = false;
    }

    void Localization::setFootwayWidth(const double footway_width) {
        this->footway_width_ = footway_width;
    }

    void Localization::setPositionFromGPS(const double lat, const double lon) {

        positions_.geo_point.latitude = lat;
        positions_.geo_point.longitude = lon;
        positions_.id = map_->getNearestPoint(lat, lon);
        positions_.is_set_from_gps = true;
        positions_.cartesian_pose.position.x = map_->getCalculator()->getCoordinateX(positions_.geo_point);
        positions_.cartesian_pose.position.y = map_->getCalculator()->getCoordinateY(positions_.geo_point);
        positions_.cartesian_pose.orientation = tf::createQuaternionMsgFromYaw(map_->getCalculator()->getBearing(positions_.geo_point));

        //checking distance to the nearest point
        double dist = getDistanceFromWay();
        if (dist > map_->getInterpolationMaxDistance())
            ROS_WARN("OSM planner: The %s coordinates is %f m out of the way", name_.c_str(), dist);
    }

    void Localization::setPositionFromPose(const geometry_msgs::Pose &pose) {

        positions_.id = map_->getNearestPointXY(pose.position.x, pose.position.y);
        positions_.geo_point = map_->getNodeByID(positions_.id);     // Set coordination of nearest point on the map
        positions_.cartesian_pose = pose;
        positions_.is_set_from_gps = false;

        //checking distance to the nearest point
        double dist = getDistanceFromWay();
        if (dist > map_->getInterpolationMaxDistance())
            ROS_WARN("OSM planner: The %s coordinates is %f m out of the way", name_.c_str(), dist);
    }

    double Localization::getFootwayWidth(){
        return footway_width_;
    }

    bool Localization::isPositionFromGps() {
        return positions_.is_set_from_gps;
    }

    int Localization::getPositionNodeID(){
        return positions_.id;
    }

    Parser::OSM_NODE Localization::getGeoPoint() {
        return positions_.geo_point;
    }

    geometry_msgs::Pose Localization::getPose() {
        return positions_.cartesian_pose;
    }

    double Localization::getDistanceFromWay(){

        Parser::OSM_NODE node = map_->getNodeByID(positions_.id);

        if (positions_.is_set_from_gps){
            return map_->getCalculator()->getDistance(positions_.geo_point, node) - footway_width_;

        } else{
            double x = map_->getCalculator()->getCoordinateX(node);
            double y = map_->getCalculator()->getCoordinateY(node);
            return sqrt(pow(x - positions_.cartesian_pose.position.x, 2.0) + pow(y - positions_.cartesian_pose.position.y, 2.0)) - footway_width_;
        }
    }
}