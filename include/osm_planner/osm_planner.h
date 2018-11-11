#include <ros/ros.h>
#include <osm_planner/path_finder_algorithm/dijkstra.h>
#include <osm_planner/osm_parser.h>
#include <osm_planner/osm_localization.h>
#include <osm_planner/newTarget.h>
#include <osm_planner/cancelledPoint.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

//navigation plugin
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_broadcaster.h>

namespace osm_planner {

    const static int FROM_SERVICE = 0;
    const static int FIRST_POINT = 1;
    const static int RANDOM_POINT = 2;
    const static int FROM_PARAM = 3;

    class Planner: public nav_core::BaseGlobalPlanner{
    public:

        Planner();
        void initialize();

        /** overriden classes from interface nav_core::BaseGlobalPlanner **/
        Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        int makePlan(double target_latitude, double target_longitude);

    protected:

        //Class for localization on the map
          std::shared_ptr<Localization> localization_source_;
        std::shared_ptr<Localization> localization_target_;
        ros::NodeHandle n;

        //make plan from source to target
        int planning(int sourceID, int targetID);

        //deleted selected point id on the path
        int cancelPoint(int pointID);


    private:

        struct OsmPath{
            nav_msgs::Path nav_path;
            std::vector<int> node_path;
        };

        std::shared_ptr<Parser> map;
        std::shared_ptr<path_finder_algorithm::PathFinderBase> path_finder_;

        bool initialized_ros;

        /*Publisher*/
        ros::Publisher shortest_path_pub;

        //msgs for shortest path
        OsmPath shortest_path_;

        /* Services */
        ros::ServiceServer cancel_point_service;
        ros::ServiceServer drawing_route_service;

        //callbacks
        bool cancelPointCallback(osm_planner::cancelledPoint::Request &req, osm_planner::cancelledPoint::Response &res);
        bool drawingRouteCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    };
}