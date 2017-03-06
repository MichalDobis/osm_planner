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
PLUGINLIB_EXPORT_CLASS(osm::Planner, nav_core::BaseGlobalPlanner);

namespace osm {

    Planner::Planner() :
            osm(), dijkstra(), targetID(100), sourceID(130), initialized(false) {

        initialize();
    }

    Planner::Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
            osm(), dijkstra(), targetID(100), sourceID(130), initialized(false) {

       initialize(name, costmap_ros);
    }


    void Planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize();
    }

    void Planner::initialize(){
        //init ros topics and services
        ros::NodeHandle n;

        //source of map
        std::string file = "skuska.osm";
        n.getParam("filepath", file);
        osm.setNewMap(file);

        //for point interpolation
        n.param<double>("interpolation_max_distance", interpolation_max_distance, 1000);
        osm.setInterpolationMaxDistance(interpolation_max_distance);

        init_service = n.advertiseService("init", &Planner::initCallback, this);

        ROS_WARN("OSM planner: Waiting for init position, please call init service...");
    }

   /* void Planner::initializeMap(std::string name, double lat, double lon){

        osm.setNewMap(name);
        initializePos(lat, lon);

    }*/
    void Planner::initializePos(double lat, double lon) {

        osm.parse();
        osm.setStartPoint(lat, lon);
        sourceID = osm.getNearestPoint(lat, lon);
        //checking distance to the nearest point
        double dist = checkDistance(sourceID, lat, lon);
        if (dist > interpolation_max_distance)
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);

        osm.publishPoint(lat, lon, Parser::CURRENT_POSITION_MARKER);
        //draw paths network
        osm.publishRouteNetwork();
        initialized = true;
        ROS_INFO("OSM planner: Initialized. Waiting for request of plan...");
    }

    bool Planner::initCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res){

        initializePos(req.target.latitude, req.target.longitude);
        return true;
    }

    bool Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        plan.push_back(start);
        nav_msgs::Path *path;
        setPositionFromOdom(start.pose.position);
        planning(goal, path);

        for (int i=0; i< path->poses.size(); i++){

            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

            new_goal.pose.position.x = path->poses[i].pose.position.x;
            new_goal.pose.position.y = path->poses[i].pose.position.y;
            new_goal.pose.orientation = path->poses[i].pose.orientation;

            plan.push_back(new_goal);

        }
        plan.push_back(goal);
        return true;
    }

    int Planner::planning(double target_latitude, double target_longitude,  nav_msgs::Path *path) {


        //Reference point is not initialize, please call init service
        if (!initialized) {
            return osm_planner::newTarget::Response::NOT_INIT;
        }

        this->target_latitude = target_latitude;
        this->target_longitude = target_longitude;

        ros::Time start_time = ros::Time::now();

        targetID = osm.getNearestPoint(target_latitude, target_longitude);
        osm.publishPoint(target_latitude, target_longitude, Parser::TARGET_POSITION_MARKER);

        int result = planning(sourceID, targetID, path);
        ROS_INFO("OSM planner: Plan time %f ", (ros::Time::now() - start_time).toSec());
        return result;
    }

    int Planner::planning(const geometry_msgs::PoseStamped& goal, nav_msgs::Path *path) {

        //Reference point is not initialize, please call init service
        if (!initialized) {
            return osm_planner::newTarget::Response::NOT_INIT;
        }

        ros::Time start_time = ros::Time::now();

        targetID = osm.getNearestPointXY(goal.pose.position.x, goal.pose.position.y);
        osm.publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER);

        int result = planning(sourceID, targetID, path);
        ROS_INFO("OSM planner: Plan time %f ", (ros::Time::now() - start_time).toSec());
        return result;
    }


    int Planner::planning(int sourceID, int targetID, nav_msgs::Path *path) {

        //Reference point is not initialize, please call init service
        if (!initialized) {
            return osm_planner::newTarget::Response::NOT_INIT;
        }

        //checking distance to the nearest point
        double dist = checkDistance(targetID, target_latitude, target_longitude);
        if (dist > interpolation_max_distance) {
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);

            return osm_planner::newTarget::Response::TARGET_IS_OUT_OF_WAY;
        }

        ROS_WARN("OSM planner: Planning trajectory...");
        try {
            path = osm.publishPath(dijkstra.findShortestPath(osm.getGraphOfVertex(), sourceID, targetID), target_latitude,
                                   target_longitude);

        } catch (dijkstra_exception &e) {
            if (e.get_err_id() == dijkstra_exception::NO_PATH_FOUND) {
                ROS_ERROR("OSM planner: Planning failed... Try to call init service again");
            } else
                ROS_ERROR("OSM planner: Undefined error");
            return osm_planner::newTarget::Response::PLAN_FAILED;
        }
        return osm_planner::newTarget::Response::PLAN_OK;
    }

    int Planner::cancelPoint(int pointID) {

        //Reference point is not initialize, please call init service
        if (!initialized) {
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
        sourceID = path[pointID];   //return back to last position
        try {
            osm.publishPath(dijkstra.findShortestPath(osm.getGraphOfVertex(), sourceID, targetID), target_latitude,
                            target_longitude);

        } catch (dijkstra_exception &e) {
            if (e.get_err_id() == dijkstra_exception::NO_PATH_FOUND) {
                ROS_ERROR("OSM planner: Planning failed");
            } else
                ROS_ERROR("OSM planner: undefined error");
            return osm_planner::cancelledPoint::Response::PLAN_FAILED;
        }

        return osm_planner::newTarget::Response::PLAN_OK;
    }


    void Planner::setPositionFromGPS(double lat, double lon) {

        if (!initialized)
            return;

        sourceID = osm.getNearestPoint(lat, lon);
        osm.publishPoint(lat, lon, Parser::CURRENT_POSITION_MARKER);

        //checking distance to the nearest point
        double dist = checkDistance(sourceID, lat, lon);
        if (dist > interpolation_max_distance)
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);

    }

    void Planner::setPositionFromOdom(geometry_msgs::Point point) {

        if (!initialized) {
            return;
        }

        sourceID = osm.getNearestPointXY(point.x, point.y);
        osm.publishPoint(point, Parser::CURRENT_POSITION_MARKER);
    }


    double Planner::checkDistance(int node_id, double lat, double lon) {

        Parser::OSM_NODE node1 = osm.getNodeByID(node_id);
        Parser::OSM_NODE node2;
        node2.latitude = lat;
        node2.longitude = lon;
        return Parser::Haversine::getDistance(node1, node2);
    }
}