#include <ros/ros.h>
#include <osm_planner/dijkstra.h>
#include <osm_planner/osm_parser.h>
#include <osm_planner/newTarget.h>
#include <osm_planner/cancelledPoint.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>

//navigation plugin
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <sensor_msgs/NavSatFix.h>


namespace osm_planner {

    class Planner: public nav_core::BaseGlobalPlanner{
    public:

        const static int GEOGRAPHICS_COORDINATES = 1;
        const static int CARTEZIAN_COORDINATES = 2;

        Planner();
        void initialize();

        /** overriden classes from interface nav_core::BaseGlobalPlanner **/
        Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan
        );

        //void initializeMap(std::string name, double lat, double lon);
        void initializePos(double lat, double lon);

        int planning(double target_latitude, double target_longitude, nav_msgs::Path *path);
        int planning(const geometry_msgs::PoseStamped& goal, nav_msgs::Path *path);
        int planning(int sourceID, int targetID, nav_msgs::Path *path);
        int cancelPoint(int pointID);

        void setPositionFromGPS(double lat, double lon);
        void setPositionFromOdom(geometry_msgs::Point point);

    private:

        Parser osm;
        Dijkstra dijkstra;

        int sourceID;
        int targetID;

        bool initialized;

        double target_longitude;
        double target_latitude;

        /* Subscribers */
        ros::Subscriber position_sub;

        /* Services */
        ros::ServiceServer init_service;

        double interpolation_max_distance;

        bool initCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res);
        double checkDistance(int node_id, double lat, double lon);

    };
}