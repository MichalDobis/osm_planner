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
            n("~/Planner") {

        path_finder_ = std::make_shared<osm_planner::path_finder_algorithm::Dijkstra>();
        map = std::make_shared<osm_planner::Parser>();
        localization_source_ = std::make_shared<osm_planner::Localization>(map, "source");
        localization_target_ = std::make_shared<osm_planner::Localization>(map, "target");

        initialized_ros = false;
        initialize();
    }

    Planner::Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
            n("~"+name) {

        path_finder_ = std::make_shared<osm_planner::path_finder_algorithm::Dijkstra>();
        map = std::make_shared<osm_planner::Parser>();
        localization_source_ = std::make_shared<osm_planner::Localization>(map, "source");
        localization_target_ = std::make_shared<osm_planner::Localization>(map, "target");

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

            std::string topic_name;
            n.param<std::string>("topic_shortest_path", topic_name, "/shortest_path");

            //publishers
            shortest_path_pub = n.advertise<nav_msgs::Path>(topic_name, 10);

            //services
             cancel_point_service = n.advertiseService("cancel_point", &Planner::cancelPointCallback, this);
            drawing_route_service = n.advertiseService("draw_route", &Planner::drawingRouteCallback, this);

            initialized_ros = true;

            //Debug param
            int set_origin_pose;
            double origin_lat, origin_lon;
            n.param<int>("set_origin_pose", set_origin_pose, 0);
            n.param<double>("origin_latitude", origin_lat, 0);
            n.param<double>("origin_longitude",origin_lon, 0);

            // Get params for map and parse
            //source of map
            std::string file = "skuska.osm";
            n.getParam("osm_map_path", file);
            map->setNewMap(file);

            std::vector<std::string> types_of_ways;
            n.getParam("filter_of_ways",types_of_ways);
            map->setTypeOfWays(types_of_ways);

            //Set the density of points
            double interpolation_max_distance;
            n.param<double>("interpolation_max_distance", interpolation_max_distance, 1000);
            map->setInterpolationMaxDistance(interpolation_max_distance);

            double footway_width;
            n.param<double>("footway_width", footway_width, 0);
            localization_source_->setFootwayWidth(footway_width);
            localization_target_->setFootwayWidth(footway_width);

            map->parse();

            // Find origin and set them
            Parser::OSM_NODE origin;
            // TODO param FROM_SERVICE uz nie je supportovany
            switch (set_origin_pose) {
                case FIRST_POINT:
                    map->setStartPoint(0);
                    break;
                case RANDOM_POINT:
                    map->setRandomStartPoint();
                    break;
                case FROM_PARAM:
                    // Parse map and set origin
                    map->getCalculator()->setOrigin(origin_lat, origin_lon);
                    break;
                default:
                    ROS_ERROR("Bad value of param set_origin_pose");
                  //  throw std::exception("Bad value of param set_origin_pose");
            }

            map->publishRouteNetwork();
            ROS_INFO("OSM planner: Initialized. Waiting for request of plan...");
        }
    }

    //-------------------------------------------------------------//
    //-------------MAKE PLAN from cartesian coordinates------------//
    //-------------------------------------------------------------//

    bool Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        if (!initialized_ros) {
            ROS_ERROR("OSM PLANNER: Reference point is not initialize, please call init service");
            return false;
        }

        //Set the start pose to plan
        plan.push_back(start);

        //localization of nearest point on the footway
        localization_source_->setPositionFromPose(start.pose);
        map->publishPoint(start.pose.position, Parser::CURRENT_POSITION_MARKER, 1.0, start.pose.orientation);


        //compute distance between start and goal
        double dist_x = start.pose.position.x - goal.pose.position.x;
        double dist_y = start.pose.position.y - goal.pose.position.y;
        double startGoalDist = sqrt(pow(dist_x, 2.0) + pow(dist_y, 2.0));

        //If distance between start and goal pose is lower as footway width then skip the planning on the osm map
        if (startGoalDist <  localization_source_->getFootwayWidth() + localization_source_->getDistanceFromWay()){
            plan.push_back(goal);
            shortest_path_.nav_path.poses.clear();
            shortest_path_.nav_path.poses.push_back(start);
            shortest_path_.nav_path.poses.push_back(goal);
            shortest_path_pub.publish(shortest_path_.nav_path);
            map->publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER, 1.0, goal.pose.orientation);
            return true;
        }

        //set the nearest point as target and save new target point
        localization_target_->setPositionFromPose(goal.pose);

        //draw target point
        map->publishPoint(goal.pose.position, Parser::TARGET_POSITION_MARKER, 1.0, goal.pose.orientation);


       ///start planning, the Path is obtaining in global variable nav_msgs::Path path
        int result = planning(localization_source_->getPositionNodeID(), localization_target_->getPositionNodeID());

        //check the result of planning
          if (result == osm_planner::newTarget::Response::NOT_INIT || result == osm_planner::newTarget::Response::PLAN_FAILED)
            return false;

          // Convert to geometry_msgs::PoseStamped
        for (int i=1; i< shortest_path_.nav_path.poses.size(); i++){

            geometry_msgs::PoseStamped new_goal = goal;
            new_goal.pose.position.x = shortest_path_.nav_path.poses[i].pose.position.x;
            new_goal.pose.position.y = shortest_path_.nav_path.poses[i].pose.position.y;
            new_goal.pose.orientation = shortest_path_.nav_path.poses[i].pose.orientation;
            plan.push_back(new_goal);
        }

        //add end (target) point
        shortest_path_.nav_path.poses.push_back(goal);
        shortest_path_pub.publish(shortest_path_.nav_path);
        plan.push_back(goal);

        return true;
    }

    //-------------------------------------------------------------//
    //-----------MAKE PLAN from geographics coordinates------------//
    //-------------------------------------------------------------//

    int Planner::makePlan(double target_latitude, double target_longitude) {

        //Reference point is not initialize, please call init service
        if (!initialized_ros) {
            return osm_planner::newTarget::Response::NOT_INIT;
        }

        localization_target_->setPositionFromGPS(target_latitude, target_longitude);

        //publish start and goal marker
        if (localization_source_->isPositionFromGps()) {
            map->publishPoint(localization_source_->getGeoPoint(), Parser::CURRENT_POSITION_MARKER, 1.0);
        } else{
            map->publishPoint(localization_source_->getPose().position, Parser::CURRENT_POSITION_MARKER, 1.0);
        }
        map->publishPoint(target_latitude, target_longitude, Parser::TARGET_POSITION_MARKER, 1.0);

       int result = planning(localization_source_->getPositionNodeID(), localization_target_->getPositionNodeID());

        //add end (target) point
        geometry_msgs::PoseStamped end;
        end.pose = localization_target_->getPose();
        end.header.frame_id = map->getMapFrameName();
        end.header.stamp = ros::Time::now();
        shortest_path_.nav_path.poses.push_back(end);
        shortest_path_pub.publish(shortest_path_.nav_path);
        return result;
        }

    /*--------------------PROTECTED FUNCTIONS---------------------*/


    //-------------------------------------------------------------//
    //-----------------MAKE PLAN from osm id's---------------------//
    //-------------------------------------------------------------//

    int Planner::planning(int sourceID, int targetID) {

        //Reference point is not initialize, please call init service
        if (!initialized_ros) {
            return osm_planner::newTarget::Response::NOT_INIT;
        }

        ROS_INFO("OSM planner: Planning trajectory...");
        ros::Time start_time = ros::Time::now();

        try {
            shortest_path_.node_path = path_finder_->findShortestPath(map->getGraphOfVertex(), sourceID, targetID);
            shortest_path_.nav_path = map->getPath(shortest_path_.node_path);

            ROS_INFO("OSM planner: Time of planning: %f ", (ros::Time::now() - start_time).toSec());

        } catch (path_finder_algorithm::PathFinderException &e) {
            if (e.getErrId() == path_finder_algorithm::PathFinderException::NO_PATH_FOUND) {
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
        if (!initialized_ros) {
            return osm_planner::cancelledPoint::Response::NOT_INIT;
        }

        //get current shortest path - vector of osm nodes IDs

        //if index is greater than array size
        if (pointID >= shortest_path_.node_path.size()) {
            return osm_planner::cancelledPoint::Response::BAD_INDEX;
        }

        //for drawing deleted path
        std::vector<int> refused_path(2);
        refused_path[0] = shortest_path_.node_path[pointID];
        refused_path[1] = shortest_path_.node_path[pointID + 1];
        map->publishRefusedPath(refused_path);

        //delete edge between two osm nodes
        map->deleteEdgeOnGraph(shortest_path_.node_path[pointID], shortest_path_.node_path[pointID + 1]);

        try {
            shortest_path_.node_path = path_finder_->findShortestPath(map->getGraphOfVertex(), shortest_path_.node_path[pointID], localization_target_->getPositionNodeID());
            this->shortest_path_.nav_path = map->getPath(shortest_path_.node_path);
            geometry_msgs::PoseStamped goal;
            goal.pose = localization_target_->getPose();
            goal.header.frame_id = map->getMapFrameName();
            goal.header.stamp = ros::Time::now();
            this->shortest_path_.nav_path.poses.push_back(goal);
            shortest_path_pub.publish(this->shortest_path_.nav_path);

        } catch (osm_planner::path_finder_algorithm::PathFinderException &e) {
            if (e.getErrId() == path_finder_algorithm::PathFinderException::NO_PATH_FOUND) {
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

    bool Planner::drawingRouteCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

        map->publishRouteNetwork();
        shortest_path_pub.publish(shortest_path_.nav_path);
        return true;
    }

}

