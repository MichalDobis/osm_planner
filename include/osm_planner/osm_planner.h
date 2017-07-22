#include <ros/ros.h>
#include <osm_planner/dijkstra.h>
#include <osm_planner/osm_parser.h>
#include <osm_planner/newTarget.h>
#include <osm_planner/cancelledPoint.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

//navigation plugin
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_broadcaster.h>

namespace osm_planner {

    class Planner: public nav_core::BaseGlobalPlanner{
    public:

        typedef struct point{
            int id;
            Parser::OSM_NODE geoPoint;
            geometry_msgs::PoseStamped cartesianPoint;
        }POINT;

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

        //Before start make plan, this function must be call
        void initializePos(double lat, double lon, double bearing);
        void initializePos();

        //make plan from source to target
        int planning(int sourceID, int targetID);

        //deleted selected point id on the path
        int cancelPoint(int pointID);

        //update pose
        bool updatePoseFromTF(); //from tf
        void setPositionFromGPS(double lat, double lon, double accuracy);        //from gps
        void setPositionFromOdom(geometry_msgs::Point point);  //from odom

    private:

        Parser osm;
        Dijkstra dijkstra;

        bool initialized_ros;
        bool initialized_position;

        POINT source;
        POINT target;

        //global ros parameters
        bool use_tf;
      //  bool update_rotation;
        std::string map_frame, base_link_frame;//, local_map_frame;
        double interpolation_max_distance;

        /*Publisher*/
        ros::Publisher shortest_path_pub;
     //   ros::Publisher utm_init_pub;

        /* Subscribers */
        ros::Subscriber gps_sub;

        //msgs for shortest path
        nav_msgs::Path path;
        ros::Publisher gps_odom_pub; //debug topic - calculate odometry from gps

        /* Services */
        ros::ServiceServer init_service;
        ros::ServiceServer cancel_point_service;

        //callbacks
        bool initCallback(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res);
        bool cancelPointCallback(osm_planner::cancelledPoint::Request &req, osm_planner::cancelledPoint::Response &res);
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

        double checkDistance(int node_id, double lat, double lon);
        double checkDistance(int node_id, geometry_msgs::Pose pose);
        double getAccuracy(const sensor_msgs::NavSatFix::ConstPtr& gps);

        /*tf broadcaster*/
        //tf::TransformBroadcaster br;
        //tf::Transform transform;

        //tf broadcaster thread
    //    double initial_angle;
      //  boost::shared_ptr<boost::thread> tfThread;
      //  void tfBroadcaster();
    };
}