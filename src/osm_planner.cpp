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
            osm(), dijkstra(), initialized_position(false) {

        initialized_ros = false;
        initialize();
    }

    Planner::Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
            osm(), dijkstra(), initialized_position(false) {

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
            //init ros topics and services
            ros::NodeHandle n("~/osm");

            //source of map
            std::string file = "skuska.osm";
            n.getParam("filepath", file);
            osm.setNewMap(file);

            //Set the density of points
            n.param<double>("interpolation_max_distance", interpolation_max_distance, 1000);
            osm.setInterpolationMaxDistance(interpolation_max_distance);

            std::string topic_name;
            n.param<std::string>("topic_shortest_path", topic_name, "/shortest_path");

            //names of frames
            n.param<std::string>("global_frame", map_frame, "/map");
            n.param<std::string>("local_map_frame", local_map_frame, "/rotated_map");
            n.param<std::string>("robot_base_frame", base_link_frame, "/base_link");
            n.param<bool>("use_tf", use_tf, true);
            n.param<bool>("update_rotation", update_rotation, true);

            std::string topic_gps_name;
            n.param<std::string>("topic_gps_name", topic_gps_name, "/position");

            //subscribers
            gps_sub = n.subscribe(topic_gps_name, 1, &Planner::gpsCallback, this);

            //publishers
            shortest_path_pub = n.advertise<nav_msgs::Path>(topic_name, 10);
            utm_init_pub = n.advertise<sensor_msgs::NavSatFix>("/utm/init", 10);

            //services
            init_service = n.advertiseService("init", &Planner::initCallback, this);
            cancel_point_service = n.advertiseService("cancel_point", &Planner::cancelPointCallback, this);

            initialized_ros = true;

            //Debug param
            bool set_random_pose;
            n.param<bool>("set_random_pose", set_random_pose, false);
            if (set_random_pose)
                initializePos();
            else
                ROS_WARN("OSM planner: Waiting for init position, please call init service...");
        }
    }

    //-------------------------------------------------------------//
    //-------------MAKE PLAN from cartesian coordinates------------//
    //-------------------------------------------------------------//

    bool Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        if (!initialized_position) {
            ROS_ERROR("OSM PLANNER: Reference point is not initialize, please call init service");
            return false;
        }

        //set the start Pose
        setPositionFromOdom(start.pose.position);

        //set the nearest point as target and save new target point
        target.id = osm.getNearestPointXY(goal.pose.position.x, goal.pose.position.y);
        target.cartesianPoint.pose = goal.pose;

        //draw target point
        osm.publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER, 5.0);

        double dist = checkDistance(target.id, target.cartesianPoint.pose);
        if (dist > interpolation_max_distance) {
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);
           // return osm_planner::newTarget::Response::TARGET_IS_OUT_OF_WAY;
        }

        plan.push_back(start);

       ///start planning, the Path is obtaining in global variable nav_msgs::Path path
        int result = planning(source.id, target.id);

        //check the result of planning
          if (result == osm_planner::newTarget::Response::NOT_INIT || result == osm_planner::newTarget::Response::PLAN_FAILED)
            return false;

        for (int i=0; i< path.poses.size(); i++){

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
        if (!initialized_position) {
            return osm_planner::newTarget::Response::NOT_INIT;
        }

        updatePoseFromTF(); //update source point from TF

        //save new target point
        target.geoPoint.latitude = target_latitude;
        target.geoPoint.longitude = target_longitude;
        target.id = osm.getNearestPoint(target_latitude, target_longitude);
        target.cartesianPoint.pose.position.x = Parser::Haversine::getCoordinateX(osm.getStartPoint(), target.geoPoint);
        target.cartesianPoint.pose.position.y = Parser::Haversine::getCoordinateY(osm.getStartPoint(), target.geoPoint);
        target.cartesianPoint.pose.orientation = tf::createQuaternionMsgFromYaw(Parser::Haversine::getBearing(osm.getStartPoint(), target.geoPoint));

        //draw target point
        osm.publishPoint(target_latitude, target_longitude, Parser::TARGET_POSITION_MARKER, 5.0);

        //checking distance to the nearest point
        double dist = checkDistance(target.id, target.geoPoint.latitude, target.geoPoint.longitude);
        if (dist > interpolation_max_distance) {
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);

            return osm_planner::newTarget::Response::TARGET_IS_OUT_OF_WAY;
        }
       int result = planning(source.id, target.id);

        //add end (target) point
        path.poses.push_back(target.cartesianPoint);
        shortest_path_pub.publish(path);
        return result;
        }

    /*--------------------PROTECTED FUNCTIONS---------------------*/

    //-------------------------------------------------------------//
    //---------------Initialize pose from gps source---------------//
    //-------------------------------------------------------------//

    void Planner::initializePos(double lat, double lon, double bearing) {

        osm.parse();
        osm.setStartPoint(lat, lon);

        //Save the position for path planning
        source.geoPoint.latitude = lat;
        source.geoPoint.longitude = lon;
        source.id = osm.getNearestPoint(lat, lon);
        source.cartesianPoint.pose.position.x = 0;
        source.cartesianPoint.pose.position.y = 0;

        initial_angle = bearing;
        tfThread = boost::shared_ptr<boost::thread>(new boost::thread(&Planner::tfBroadcaster, this));
        usleep(500000);
        //checking distance to the nearest point
        double dist = checkDistance(source.id, lat, lon);
        if (dist > interpolation_max_distance)
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);


        //draw paths network
        osm.publishRouteNetwork();
        osm.publishPoint(lat, lon, Parser::CURRENT_POSITION_MARKER, 50.0);

        initialized_position = true;
        ROS_INFO("OSM planner: Initialized. Waiting for request of plan...");
    }

    //-------------------------------------------------------------//
    //--------Initialize pose from random gen - for debug----------//
    //-------------------------------------------------------------//
    void Planner::initializePos() {

        osm.parse();
        osm.setStartPoint();

        //Save the position for path planning
        source.geoPoint = osm.getStartPoint();
        source.id = 0;
        source.cartesianPoint.pose.position.x = 0;
        source.cartesianPoint.pose.position.y = 0;

        osm.publishPoint(source.geoPoint.latitude, source.geoPoint.longitude, Parser::CURRENT_POSITION_MARKER, 5.0);
        //draw paths network
        osm.publishRouteNetwork();
        initialized_position = true;
        ROS_INFO("OSM planner: Initialized. Waiting for request of plan...");
    }

    //-------------------------------------------------------------//
    //-----------------MAKE PLAN from osm id's---------------------//
    //-------------------------------------------------------------//

    int Planner::planning(int sourceID, int targetID) {

        //Reference point is not initialize, please call init service
        if (!initialized_position) {
            return osm_planner::newTarget::Response::NOT_INIT;
        }

        ROS_INFO("OSM planner: Planning trajectory...");
        ros::Time start_time = ros::Time::now();

        try {
            path = osm.getPath(dijkstra.findShortestPath(osm.getGraphOfVertex(), sourceID, targetID));

            ROS_INFO("OSM planner: Plan time %f ", (ros::Time::now() - start_time).toSec());

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
        if (!initialized_position) {
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
        if (!updatePoseFromTF()) {     //update source position from TF
            source.id = path[pointID];   //if source can not update from TF, return back to last position
        }

        try {

            this->path = osm.getPath(dijkstra.findShortestPath(osm.getGraphOfVertex(), source.id, target.id));
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
    //-----------------UPDATE POSE FUNCTIONS-----------------------//
    //-------------------------------------------------------------//

    bool Planner::updatePoseFromTF() {

        if (!initialized_position || !use_tf)
            return false;

        geometry_msgs::Point point;
        tf::TransformListener listener;
        tf::StampedTransform transform;

        try {
            listener.waitForTransform(base_link_frame, map_frame, ros::Time(0), ros::Duration(1));
            listener.lookupTransform(base_link_frame, map_frame, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
           ROS_WARN("OSM planner: %s. Can't update pose from TF, for that will be use the latest source point.",
                    ex.what());
        }

        tf::pointTFToMsg(transform.getOrigin(), point);

        setPositionFromOdom(point);
        return true;
    }


    void Planner::setPositionFromGPS(double lat, double lon, double accuracy) {

        if (!initialized_position)
            return;

        //update source point
        source.id = osm.getNearestPoint(lat, lon);
        source.geoPoint.latitude = lat;
        source.geoPoint.longitude = lon;
        source.cartesianPoint.pose.position.x = Parser::Haversine::getCoordinateX(osm.getStartPoint(), source.geoPoint);
        source.cartesianPoint.pose.position.y = Parser::Haversine::getCoordinateY(osm.getStartPoint(), source.geoPoint);
        source.cartesianPoint.pose.orientation = tf::createQuaternionMsgFromYaw(Parser::Haversine::getBearing(osm.getStartPoint(), source.geoPoint));

        osm.publishPoint(lat, lon, Parser::CURRENT_POSITION_MARKER, accuracy);

        //checking distance to the nearest point
        double dist = checkDistance(source.id, lat, lon);
        if (dist > interpolation_max_distance)
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);
    }

    void Planner::setPositionFromOdom(geometry_msgs::Point point) {

        if (!initialized_position) {
            return;
        }

        //update source point
        source.id = osm.getNearestPointXY(point.x, point.y);
        source.cartesianPoint.pose.position = point;
       // osm.publishPoint(point, Parser::CURRENT_POSITION_MARKER, 5.0);

        //checking distance to the nearest point
        double dist = checkDistance(source.id, source.cartesianPoint.pose);
        if (dist > interpolation_max_distance)
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);
    }

    //-------------------------------------------------------------//
    //-------------------------------------------------------------//


    /*--------------------PRIVATE FUNCTIONS---------------------*/

    bool Planner::cancelPointCallback(osm_planner::cancelledPoint::Request &req, osm_planner::cancelledPoint::Response &res){

        res.result = cancelPoint(req.pointID);
        return true;
    }

    bool Planner::initCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res){

        initializePos(req.latitude, req.longitude, req.bearing);

        sensor_msgs::NavSatFix fix;

        fix.latitude = req.latitude;
        fix.longitude = req.longitude;

        fix.header.frame_id = base_link_frame;
        utm_init_pub.publish(fix);

        return true;
    }

    void Planner::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

        if (!initialized_position)
            return;

        double cov = getAccuracy(msg);

        //todo - dorobit citanie gps vzhladom na frame
        setPositionFromGPS(msg->latitude, msg->longitude, cov);

        if (update_rotation) {

            static Parser::OSM_NODE lastPoint = osm.getStartPoint();
            static double last_cov = 0;

            if (Parser::Haversine::getDistance(lastPoint, *msg) > cov + last_cov) {

                initial_angle = Parser::Haversine::getBearing(lastPoint, *msg);
                tf::Quaternion q;
                q.setRPY(0, 0, initial_angle);
                transform.setRotation(q);

                lastPoint.latitude = msg->latitude;
                lastPoint.longitude = msg->longitude;
                last_cov = cov;
            }
            osm.publishPoint(lastPoint.latitude, lastPoint.longitude, Parser::TARGET_POSITION_MARKER, last_cov);
        }
    }

    double Planner::getAccuracy(const sensor_msgs::NavSatFix::ConstPtr& gps){

        double sum = 0;
        for (double cov: gps->position_covariance){
            sum += pow(cov, 2.0);
        }
        return sqrt(sum);
    }

    double Planner::checkDistance(int node_id, double lat, double lon) {

        Parser::OSM_NODE node1 = osm.getNodeByID(node_id);
        Parser::OSM_NODE node2;
        node2.latitude = lat;
        node2.longitude = lon;
        return Parser::Haversine::getDistance(node1, node2);
    }

    double Planner::checkDistance(int node_id, geometry_msgs::Pose pose) {

        Parser::OSM_NODE node = osm.getNodeByID(node_id);

        double x = Parser::Haversine::getCoordinateX(osm.getStartPoint(), node);
        double y = Parser::Haversine::getCoordinateY(osm.getStartPoint(), node);

        return sqrt(pow(x - pose.position.x, 2.0) + pow(y - pose.position.y, 2.0));
    }

    void Planner::tfBroadcaster(){

        //inicialize TF broadcaster
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, initial_angle);
        transform.setRotation(q);

        ros::Rate rate(10);

        while (ros::ok()){

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, local_map_frame));
            rate.sleep();
        }
    }
}

