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

        typedef struct point{
            int id;
            Parser::OSM_NODE geoPoint;
            geometry_msgs::PoseStamped cartesianPoint;
        }POINT;

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

        int makePlan(double target_latitude, double target_longitude);

    protected:

        void initializePos(double lat, double lon);

        int planning(int sourceID, int targetID);
        int cancelPoint(int pointID);

        void setPositionFromGPS(double lat, double lon);
        void setPositionFromOdom(geometry_msgs::Point point);

    private:

        Parser osm;
        Dijkstra dijkstra;

        bool initialized_ros;
        bool initialized_position;

        POINT source;
        POINT target;

        /*Publisher*/
        ros::Publisher shortest_path_pub;

        //msgs for shortest path
        nav_msgs::Path path;

        /* Services */
        ros::ServiceServer init_service;
        ros::ServiceServer cancel_point_service;

        double interpolation_max_distance;

        bool initCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res);
        bool cancelPointCallback(osm_planner::cancelledPoint::Request &req, osm_planner::cancelledPoint::Response &res);

        double checkDistance(int node_id, double lat, double lon);
        double checkDistance(int node_id, geometry_msgs::Pose pose);
    };
}