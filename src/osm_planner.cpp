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
            n.param<bool>("update_origin_pose", update_origin_pose, false);
            n.param<double>("footway_width", footway_width, 0);

            std::string topic_gps_name;
            n.param<std::string>("topic_gps_name", topic_gps_name, "/position");

            initFromGpsCallback = false;

            //subscribers
            gps_sub = n.subscribe(topic_gps_name, 1, &Planner::gpsCallback, this);

            //publishers
            shortest_path_pub = n.advertise<nav_msgs::Path>(topic_name, 10);
            gps_odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 10);

          //  utm_init_pub = n.advertise<sensor_msgs::NavSatFix>("/utm/init", 10);

            //services
            init_service = n.advertiseService("init", &Planner::initCallback, this);
            cancel_point_service = n.advertiseService("cancel_point", &Planner::cancelPointCallback, this);
            drawing_route_service = n.advertiseService("draw_route", &Planner::drawingRouteCallback, this);

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

        //Set the start pose to plan
        plan.push_back(start);

        //localization of nearest point on the footway
        setPositionFromOdom(start.pose.position);

        //check target distance from footway
        double dist = checkDistance(target.id, target.cartesianPoint.pose);
        if (dist > interpolation_max_distance) {
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);
            // return osm_planner::newTarget::Response::TARGET_IS_OUT_OF_WAY;
        }

        //compute distance between start and goal
        double dist_x = start.pose.position.x - goal.pose.position.x;
        double dist_y = start.pose.position.y - goal.pose.position.y;
        double startGoalDist = sqrt(pow(dist_x, 2.0) + pow(dist_y, 2.0));

        //If distance between start and goal pose is lower as footway width then skip the planning on the osm map
        if (startGoalDist <  footway_width + checkDistance(source.id, start.pose)){
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
        target.cartesianPoint.pose.position.x =  osm.getCalculator()->getCoordinateX(target.geoPoint);
        target.cartesianPoint.pose.position.y =  osm.getCalculator()->getCoordinateY(target.geoPoint);
        target.cartesianPoint.pose.orientation = tf::createQuaternionMsgFromYaw( osm.getCalculator()->getBearing(target.geoPoint));

        //draw target point
        osm.publishPoint(target_latitude, target_longitude, Parser::TARGET_POSITION_MARKER, 1.0, target.cartesianPoint.pose.orientation);

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

        //todo - domysliet, ze co rotovat. Ci je lepsie rotovat celu mapu a robot staticky pri inite.
        // todo - alebo mapa staticka a rototovat frame local_map, tak aby sa zrotoval robot aj s lokalnou a SLAM mapou
        osm.getCalculator()->setOffset(bearing);
        initializePos(lat, lon);
    }

    void Planner::initializePos(double lat, double lon) {

        osm.parse();
        osm.getCalculator()->setOrigin(lat, lon);

        //Save the position for path planning
        source.geoPoint.latitude = lat;
        source.geoPoint.longitude = lon;
        source.id = osm.getNearestPoint(lat, lon);
        source.cartesianPoint.pose.position.x = 0;
        source.cartesianPoint.pose.position.y = 0;

       // initial_angle = bearing;
        if (update_origin_pose && !tfThread != NULL) {
              tfThread = boost::shared_ptr<boost::thread>(new boost::thread(&Planner::tfBroadcaster, this));
                usleep(500000);
        }
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
        source.geoPoint = osm.getCalculator()->getOrigin();
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
        source.cartesianPoint.pose.position.x = osm.getCalculator()->getCoordinateX(source.geoPoint);
        source.cartesianPoint.pose.position.y = osm.getCalculator()->getCoordinateY(source.geoPoint);
        source.cartesianPoint.pose.orientation = tf::createQuaternionMsgFromYaw(osm.getCalculator()->getBearing(source.geoPoint));

        osm.publishPoint(lat, lon, Parser::CURRENT_POSITION_MARKER, accuracy);

        //checking distance to the nearest point
        double dist = checkDistance(source.id, lat, lon);
        if (dist > interpolation_max_distance)
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);

            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();


            odom.header.frame_id = base_link_frame;

            odom.child_frame_id = map_frame;

            odom.pose.pose.position.x = source.cartesianPoint.pose.position.x;
            odom.pose.pose.position.y = source.cartesianPoint.pose.position.y;
            odom.pose.pose.position.z = 0;

            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;
            odom.pose.pose.orientation.w = 1;

         /*   // Use ENU covariance to build XYZRPY covariance
            boost::array<double, 36> covariance = {{
                                                           fix->position_covariance[0],
                                                           fix->position_covariance[1],
                                                           fix->position_covariance[2],
                                                           0, 0, 0,
                                                           fix->position_covariance[3],
                                                           fix->position_covariance[4],
                                                           fix->position_covariance[5],
                                                           0, 0, 0,
                                                           fix->position_covariance[6],
                                                           fix->position_covariance[7],
                                                           fix->position_covariance[8],
                                                           0, 0, 0,
                                                           0, 0, 0, rot_cov, 0, 0,
                                                           0, 0, 0, 0, rot_cov, 0,
                                                           0, 0, 0, 0, 0, rot_cov
                                                   }};

            odom.pose.covariance = covariance;*/

            gps_odom_pub.publish(odom);
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

        //if longitude and latitude are incorrect then get initalize pose from gps topic
        if (req.longitude <= 0 && req.latitude <= 0 ) {
            initFromGpsCallback = true;
            osm.getCalculator()->setOffset(req.bearing);
        }
        else
            //If data is correct
            initializePos(req.latitude, req.longitude, req.bearing);

        return true;
    }

    bool Planner::drawingRouteCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

        osm.publishRouteNetwork();
        shortest_path_pub.publish(path);
        return true;
    }

    void Planner::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

        //If is request for initiliaze pose from gps callback
        if (initFromGpsCallback && msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX) {

            initFromGpsCallback = false;
            initializePos(msg->latitude, msg->longitude);

        }

        //if pose no initialized then cancel callback
        if (!initialized_position)
            return;

        double cov = getAccuracy(msg);

        //todo - dorobit citanie gps vzhladom na frame
        setPositionFromGPS(msg->latitude, msg->longitude, cov);

        if (update_origin_pose) {

           // static Parser::OSM_NODE lastPoint = osm.getCalculator()->getOrigin();
            static double last_cov = 100000;

            if (cov < last_cov){

                improveOrigin(msg);
                last_cov = cov;

            }

           /* if (Parser::Haversine::getDistance(lastPoint, *msg) > cov + last_cov) {

                initial_angle = Parser::Haversine::getBearing(lastPoint, *msg);
                tf::Quaternion q;
                q.setRPY(0, 0, initial_angle);
                transform.setRotation(q);

                lastPoint.latitude = msg->latitude;
                lastPoint.longitude = msg->longitude;
                last_cov = cov;
            }*/

           // osm.publishPoint(lastPoint.latitude, lastPoint.longitude, Parser::TARGET_POSITION_MARKER, last_cov);
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
        return Parser::Haversine::getDistance(node1, node2) - footway_width;
    }

    double Planner::checkDistance(int node_id, geometry_msgs::Pose pose) {

        Parser::OSM_NODE node = osm.getNodeByID(node_id);


        double x = osm.getCalculator()->getCoordinateX(node);
        double y = osm.getCalculator()->getCoordinateY(node);

        return sqrt(pow(x - pose.position.x, 2.0) + pow(y - pose.position.y, 2.0)) - footway_width;
    }


    void Planner::improveOrigin(const sensor_msgs::NavSatFix::ConstPtr& gps){

        updatePoseFromTF();

        double odom_x = source.cartesianPoint.pose.position.x;
        double odom_y = source.cartesianPoint.pose.position.y;

        double pose_from_origin_x = osm.getCalculator()->getCoordinateX(source.geoPoint);
        double pose_from_origin_y = osm.getCalculator()->getCoordinateY(source.geoPoint);

        double diff_x = pose_from_origin_x - odom_x;
        double diff_y = pose_from_origin_y - odom_y;

        ROS_WARN("improve origin pose x:%f y:%f", diff_x, diff_y);

        transform.setOrigin(tf::Vector3(diff_x, diff_y, 0));

    }

    void Planner::tfBroadcaster(){

        //inicialize TF broadcaster
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);

        ros::Rate rate(10);

        while (ros::ok()){

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, local_map_frame));
            rate.sleep();
        }
    }
}

