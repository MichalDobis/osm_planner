/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

#include <osm_planner/osm_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(osm_planner::Planner, nav_core::BaseGlobalPlanner);

namespace osm_planner {

    /*--------------------CONSTRUCTORS---------------------*/


    Planner::Planner() :
            osm(), dijkstra(), localization(&osm), n("~/Planner") {

        initialized_ros = false;
        initialize();
    }

    Planner::Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
            osm(), dijkstra(), localization(&osm), n("~"+name) {

        initialized_ros = false;
        initialize(name, costmap_ros);
    }


    /*--------------------PUBLIC FUNCTIONS---------------------*/

    void Planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

        initialize();
    }

    //-------------------------------------------------------------//
    //------------------Initialize ROS utilities-------------------//
    //-------------------------------------------------------------//

    void Planner::initialize(){

        if (!initialized_ros) {

            //source of map
            std::string file = "skuska.osm";
            n.getParam("filepath", file);
            osm.setNewMap(file);

            std::string topic_name;
            n.param<std::string>("topic_shortest_path", topic_name, "/shortest_path");

            std::string topic_gps_name;
            n.param<std::string>("topic_gps_name", topic_gps_name, "/position");

            n.param<bool>("use_map_rotation", use_map_rotation, true);
            initFromGpsCallback = false;

            //subscribers
            gps_sub = n.subscribe(topic_gps_name, 1, &Planner::gpsCallback, this);

            //publishers
            shortest_path_pub = n.advertise<nav_msgs::Path>(topic_name, 10);
            gps_odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 10);

          //  utm_init_pub = n.advertise<sensor_msgs::NavSatFix>("/utm/init", 10);

            //services
            init_service = n.advertiseService("init", &Planner::initCallback, this);
            computeBearing = n.advertiseService("compute_bearing", &Planner::computeBearingCallback, this);
            cancel_point_service = n.advertiseService("cancel_point", &Planner::cancelPointCallback, this);
            drawing_route_service = n.advertiseService("draw_route", &Planner::drawingRouteCallback, this);

            initialized_ros = true;

            localization.initialize();

            //Debug param
            bool set_random_pose;
            n.param<bool>("set_random_pose", set_random_pose, false);
            if (set_random_pose)
                localization.initializePos();
            else
                ROS_WARN("OSM planner: Waiting for init position, please call init service...");
        }
    }

    //-------------------------------------------------------------//
    //-------------MAKE PLAN from cartesian coordinates------------//
    //-------------------------------------------------------------//

    bool Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        if (!localization.isInitialized()) {
            ROS_ERROR("OSM PLANNER: Reference point is not initialize, please call init service");
            return false;
        }

        //Set the start pose to plan
        plan.push_back(start);

        //localization of nearest point on the footway
        localization.setPositionFromOdom(start.pose.position);

        //check target distance from footway
        localization.checkDistance(target.id, target.cartesianPoint.pose);

        //compute distance between start and goal
        double dist_x = start.pose.position.x - goal.pose.position.x;
        double dist_y = start.pose.position.y - goal.pose.position.y;
        double startGoalDist = sqrt(pow(dist_x, 2.0) + pow(dist_y, 2.0));

        //If distance between start and goal pose is lower as footway width then skip the planning on the osm map
        if (startGoalDist <  localization.getFootwayWidth() + localization.checkDistance(localization.getCurrentPosition()->id, start.pose)){
            plan.push_back(goal);
            path.poses.clear();
            path.poses.push_back(start);
            path.poses.push_back(goal);
            shortest_path_pub.publish(path);
            osm.publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER, 1.0, goal.pose.orientation);
            return true;
        }

        //set the nearest point as target and save new target point
        target.id = osm.getNearestPointXY(goal.pose.position.x, goal.pose.position.y);
        target.cartesianPoint.pose = goal.pose;

        //draw target point
        osm.publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER, 1.0, goal.pose.orientation);


       ///start planning, the Path is obtaining in global variable nav_msgs::Path path
        int result = planning(localization.getCurrentPosition()->id, target.id);

        //check the result of planning
          if (result == osm_planner::newTarget::Response::NOT_INIT || result == osm_planner::newTarget::Response::PLAN_FAILED)
            return false;

        for (int i=1; i< path.poses.size(); i++){

            geometry_msgs::PoseStamped new_goal = goal;

            new_goal.pose.position.x = path.poses[i].pose.position.x;
            new_goal.pose.position.y = path.poses[i].pose.position.y;
            new_goal.pose.orientation = path.poses[i].pose.orientation;

            plan.push_back(new_goal);
        }

        //add end (target) point
        path.poses.push_back(goal);
        shortest_path_pub.publish(path);
        plan.push_back(goal);

        return true;
    }

    //-------------------------------------------------------------//
    //-----------MAKE PLAN from geographics coordinates------------//
    //-------------------------------------------------------------//

    int Planner::makePlan(double target_latitude, double target_longitude) {

        //Reference point is not initialize, please call init service
        if (!localization.isInitialized()) {
            return osm_planner::newTarget::Response::NOT_INIT;
        }

        localization.updatePoseFromTF(); //update source point from TF

        //save new target point
        target.geoPoint.latitude = target_latitude;
        target.geoPoint.longitude = target_longitude;
        target.id = osm.getNearestPoint(target_latitude, target_longitude);
        target.cartesianPoint.pose.position.x =  osm.getCalculator()->getCoordinateX(target.geoPoint);
        target.cartesianPoint.pose.position.y =  osm.getCalculator()->getCoordinateY(target.geoPoint);
        target.cartesianPoint.pose.orientation = tf::createQuaternionMsgFromYaw( osm.getCalculator()->getBearing(target.geoPoint));

        //draw target point
        osm.publishPoint(target_latitude, target_longitude, Parser::TARGET_POSITION_MARKER, 1.0, target.cartesianPoint.pose.orientation);

        //checking distance to the nearest point
        localization.checkDistance(target.id, target.geoPoint.latitude, target.geoPoint.longitude);

       int result = planning(localization.getCurrentPosition()->id, target.id);

        //add end (target) point
        path.poses.push_back(target.cartesianPoint);
        shortest_path_pub.publish(path);
        return result;
        }

    /*--------------------PROTECTED FUNCTIONS---------------------*/


    //-------------------------------------------------------------//
    //-----------------MAKE PLAN from osm id's---------------------//
    //-------------------------------------------------------------//

    int Planner::planning(int sourceID, int targetID) {

        //Reference point is not initialize, please call init service
        if (!localization.isInitialized()) {
            return osm_planner::newTarget::Response::NOT_INIT;
        }

        ROS_INFO("OSM planner: Planning trajectory...");
        ros::Time start_time = ros::Time::now();

        try {
            path = osm.getPath(dijkstra.findShortestPath(osm.getGraphOfVertex(), sourceID, targetID));

            ROS_INFO("OSM planner: Time of planning: %f ", (ros::Time::now() - start_time).toSec());

        } catch (dijkstra_exception &e) {
            if (e.get_err_id() == dijkstra_exception::NO_PATH_FOUND) {
                ROS_ERROR("OSM planner: Make plan failed...");
            } else
                ROS_ERROR("OSM planner: Undefined error");
            return osm_planner::newTarget::Response::PLAN_FAILED;
        }
        return osm_planner::newTarget::Response::PLAN_OK;
    }

    //-------------------------------------------------------------//
    //-------------Refuse point and make plan again----------------//
    //-------------------------------------------------------------//

    int Planner::cancelPoint(int pointID) {

        //Reference point is not initialize, please call init service
        if (!localization.isInitialized()) {
            return osm_planner::cancelledPoint::Response::NOT_INIT;
        }

        //get current shortest path - vector of osm nodes IDs
        std::vector<int> path = dijkstra.getSolution();

        //if index is greater than array size
        if (pointID >= path.size()) {
            return osm_planner::cancelledPoint::Response::BAD_INDEX;
        }

        //for drawing deleted path
        std::vector<int> refused_path(2);
        refused_path[0] = path[pointID];
        refused_path[1] = path[pointID + 1];
        osm.publishRefusedPath(refused_path);

        //delete edge between two osm nodes
        osm.deleteEdgeOnGraph(path[pointID], path[pointID + 1]);

        //planning shorest path
        if (!localization.updatePoseFromTF()) {     //update source position from TF
            localization.getCurrentPosition()->id = path[pointID];   //if source can not update from TF, return back to last position
        }

        try {

            this->path = osm.getPath(dijkstra.findShortestPath(osm.getGraphOfVertex(), localization.getCurrentPosition()->id, target.id));
            this->path.poses.push_back(target.cartesianPoint);
            shortest_path_pub.publish(this->path);

        } catch (dijkstra_exception &e) {
            if (e.get_err_id() == dijkstra_exception::NO_PATH_FOUND) {
                ROS_ERROR("OSM planner: Make plan failed");
            } else
                ROS_ERROR("OSM planner: Undefined error");
            return osm_planner::cancelledPoint::Response::PLAN_FAILED;
        }

        return osm_planner::newTarget::Response::PLAN_OK;
    }

    //-------------------------------------------------------------//
    //-------------------------------------------------------------//


    /*--------------------PRIVATE FUNCTIONS---------------------*/

    bool Planner::cancelPointCallback(osm_planner::cancelledPoint::Request &req, osm_planner::cancelledPoint::Response &res){

        res.result = cancelPoint(req.pointID);
        return true;
    }


    bool Planner::initCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res){

        //if longitude and latitude are incorrect then get initalize pose from gps topic
        if (req.longitude <= 0 && req.latitude <= 0 ) initFromGpsCallback = true;
        else localization.initializePos(req.latitude, req.longitude);

        // if isn't set the flag update_tf_pose_from_gps, then set rotation of map
        // else set rotation of tf
       if  (use_map_rotation) osm.getCalculator()->setOffset(req.bearing);
        else localization.getTF()->improveTfRotation(req.bearing);

     //   localization.getTF()->setTfRotation(req.bearing);
        return true;
    }

    bool Planner::computeBearingCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res){

        if (!localization.isInitialized())
            return true;

        osm_planner::Parser::OSM_NODE currentPose = localization.getCurrentPosition()->geoPoint;
        double angle = osm.getCalculator()->getBearing(currentPose);
        ROS_WARN("set bearing %f", angle);
        localization.getTF()->setTfRotation(angle);
        return true;
    }

    bool Planner::drawingRouteCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

        osm.publishRouteNetwork();
        shortest_path_pub.publish(path);
        return true;
    }

    void Planner::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

        if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
            return;

        //If is request for initiliaze pose from gps callback
        if (initFromGpsCallback) {

            initFromGpsCallback = false;
            localization.initializePos(msg->latitude, msg->longitude);

        }

        localization.setPositionFromGPS(msg);

        //gps to odom publisher
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = localization.getTF()->getBaseLinkFrame();
        odom.child_frame_id = localization.getTF()->getMapFrame();

        odom.pose.pose.position.x = localization.getCurrentPosition()->cartesianPoint.pose.position.x;
        odom.pose.pose.position.y = localization.getCurrentPosition()->cartesianPoint.pose.position.y;
        odom.pose.pose.position.z = 0;

        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = 0;
        odom.pose.pose.orientation.w = 1;

        gps_odom_pub.publish(odom);
    }

}

