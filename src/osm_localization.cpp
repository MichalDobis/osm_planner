//
// Created by michal on 30.12.2016.
//
#include <osm_planner/osm_localization.h>



namespace osm_planner {


    Localization::Localization(osm_planner::Parser *map) : tfHandler(map->getCalculator()) {

        this->map = map;
        initialized_position = false;
        initialized_ros = false;

    }

    Localization::POINT *Localization::getCurrentPosition(){

        return &source;
    }

    TfHandler *Localization::getTF(){

        return &tfHandler;
    }

    double Localization::getFootwayWidth(){
        return footway_width;
    }

    bool Localization::isInitialized() {

        return initialized_ros && initialized_position;
    }

    void Localization::initialize(){

        if (!initialized_ros) {
            //init ros topics and services
            ros::NodeHandle n("~/osm");

            //Set the density of points
            n.param<double>("interpolation_max_distance", interpolation_max_distance, 1000);
            map->setInterpolationMaxDistance(interpolation_max_distance);

            std::string topic_name;
            n.param<std::string>("topic_shortest_path", topic_name, "/shortest_path");

            //names of frames
            std::string map_frame, local_map_frame, base_link_frame;

            n.param<std::string>("global_frame", map_frame, "/map");
            n.param<std::string>("local_map_frame", local_map_frame, "/rotated_map");
            n.param<std::string>("robot_base_frame", base_link_frame, "/base_link");
            tfHandler.setFrames(map_frame, local_map_frame, base_link_frame);

            n.param<int>("update_tf_pose_from_gps", update_tf_pose_from_gps, 2);
            n.param<double>("footway_width", footway_width, 0);


            //create tf broadcaster thread
            //   if (update_tf_pose_from_gps) {
            tfHandler.initThread();
            //  }


            initialized_ros = true;

        }

    }
    //-------------------------------------------------------------//
    //---------------Initialize pose from gps source---------------//
    //-------------------------------------------------------------//

    /* void Planner::initializePos(double lat, double lon, double bearing) {

         //todo - domysliet, ze co rotovat. Ci je lepsie rotovat celu mapu a robot staticky pri inite.
         // todo - alebo mapa staticka a rototovat frame local_map, tak aby sa zrotoval robot aj s lokalnou a SLAM mapou
         osm.getCalculator()->setOffset(bearing);
         initializePos(lat, lon);
     }*/

    void Localization::initializePos(double lat, double lon) {

        //if position is initialized then do nothing
        if (initialized_position)
            return;

        map->parse();
        map->getCalculator()->setOrigin(lat, lon);

        //Save the position for path planning
        source.geoPoint.latitude = lat;
        source.geoPoint.longitude = lon;
        source.id = map->getNearestPoint(lat, lon);
        source.cartesianPoint.pose.position.x = 0;
        source.cartesianPoint.pose.position.y = 0;
        //create tf broadcaster thread


        //checking distance to the nearest point
        double dist = checkDistance(source.id, lat, lon);
        if (dist > interpolation_max_distance)
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);

        //draw paths network
        map->publishRouteNetwork();
        map->publishPoint(lat, lon, Parser::CURRENT_POSITION_MARKER, 50.0);

        initialized_position = true;
        ROS_INFO("OSM planner: Initialized. Waiting for request of plan...");
    }

    //-------------------------------------------------------------//
    //--------Initialize pose from random gen - for debug----------//
    //-------------------------------------------------------------//
    void Localization::initializePos() {

        //if position is initialized then do nothing
        if (initialized_position)
            return;

        map->parse();
        map->setStartPoint();

        //Save the position for path planning
        source.geoPoint = map->getCalculator()->getOrigin();
        source.id = 0;
        source.cartesianPoint.pose.position.x = 0;
        source.cartesianPoint.pose.position.y = 0;

        map->publishPoint(source.geoPoint.latitude, source.geoPoint.longitude, Parser::CURRENT_POSITION_MARKER, 5.0);
        //draw paths network
        map->publishRouteNetwork();
        initialized_position = true;
        ROS_INFO("OSM planner: Initialized. Waiting for request of plan...");

    }

    //-------------------------------------------------------------//
    //-----------------UPDATE POSE FUNCTIONS-----------------------//
    //-------------------------------------------------------------//

    bool Localization::updatePoseFromTF() {

        if (!initialized_position)
            return false;

        setPositionFromOdom(tfHandler.getPoseFromTF(tfHandler.getMapFrame()));
    }

    bool Localization::setPositionFromGPS(const sensor_msgs::NavSatFix::ConstPtr& msg) {

        if (!initialized_position)
            return false;

        //update source point
        source.id = map->getNearestPoint(msg->latitude, msg->longitude);
        source.geoPoint.latitude = msg->latitude;
        source.geoPoint.longitude = msg->longitude;
        source.cartesianPoint.pose.position.x = map->getCalculator()->getCoordinateX(source.geoPoint);
        source.cartesianPoint.pose.position.y = map->getCalculator()->getCoordinateY(source.geoPoint);
        source.cartesianPoint.pose.orientation = tf::createQuaternionMsgFromYaw(map->getCalculator()->getBearing(source.geoPoint));

        //checking distance to the nearest point
        double dist = checkDistance(source.id, msg->latitude, msg->longitude);
        if (dist > interpolation_max_distance)
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);

        double cov = getAccuracy(msg);

        map->publishPoint(msg->latitude, msg->longitude, Parser::CURRENT_POSITION_MARKER, cov);

        switch (update_tf_pose_from_gps){
            case 0:
                break;
            case 1:
                static double last_cov = 100000;

                if (cov < last_cov){

                    tfHandler.improveTfPoseFromGPS(msg);
                    last_cov = cov;
                }
                break;
            case 2:
                tfHandler.improveTfPoseFromGPS(msg);
                break;
        }

        return true;
    }

    void Localization::setPositionFromOdom(geometry_msgs::Point point) {

        if (!initialized_position) {
            return;
        }

        //update source point
        source.id = map->getNearestPointXY(point.x, point.y);
        source.cartesianPoint.pose.position = point;
        // osm.publishPoint(point, Parser::CURRENT_POSITION_MARKER, 5.0);

        //checking distance to the nearest point
        double dist = checkDistance(source.id, source.cartesianPoint.pose);
        if (dist > interpolation_max_distance)
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);
    }


    double Localization::getAccuracy(const sensor_msgs::NavSatFix::ConstPtr& gps){

        double sum = 0;
        for (double cov: gps->position_covariance){
            sum += pow(cov, 2.0);
        }
        return sqrt(sum);
    }

    double Localization::checkDistance(int node_id, double lat, double lon) {

        Parser::OSM_NODE node1 = map->getNodeByID(node_id);
        Parser::OSM_NODE node2;
        node2.latitude = lat;
        node2.longitude = lon;

        double dist = Parser::Haversine::getDistance(node1, node2) - footway_width;

        if (dist > interpolation_max_distance) {
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);
        }

        return dist;
    }

    double Localization::checkDistance(int node_id, geometry_msgs::Pose pose) {

        Parser::OSM_NODE node = map->getNodeByID(node_id);


        double x = map->getCalculator()->getCoordinateX(node);
        double y = map->getCalculator()->getCoordinateY(node);

        double dist = sqrt(pow(x - pose.position.x, 2.0) + pow(y - pose.position.y, 2.0)) - footway_width;

        if (dist > interpolation_max_distance) {
            ROS_WARN("OSM planner: The coordinates is %f m out of the way", dist);
        }

        return dist;
    }

    TfHandler::TfHandler(osm_planner::Parser::Haversine *calculator) {

        this->calculator = calculator;
    }

    void TfHandler::setFrames(std::string map_frame, std::string local_map_frame, std::string base_link_frame) {

        this->map_frame = map_frame;
        this->base_link_frame = base_link_frame;
        this->local_map_frame = local_map_frame;

    }

    void TfHandler::initThread(){

        tfThread = boost::shared_ptr<boost::thread>(new boost::thread(&TfHandler::tfBroadcaster, this));
        usleep(500000);

    }

    geometry_msgs::Point TfHandler::getPoseFromTF(std::string map_link) {


        geometry_msgs::Point point;
        tf::TransformListener listener;
        tf::StampedTransform transform;

        try {
            listener.waitForTransform(base_link_frame, map_link, ros::Time(0), ros::Duration(1));
            listener.lookupTransform(base_link_frame, map_link, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_WARN("OSM planner: %s. Can't update pose from TF, for that will be use the latest source point.",
                     ex.what());
        }

        tf::pointTFToMsg(transform.getOrigin(), point);

        return point;
    }

    void TfHandler::improveTfPoseFromGPS(const sensor_msgs::NavSatFix::ConstPtr& gps){

        static double diff_x = 0;
        static double diff_y = 0;

        geometry_msgs::Point positionFromTF = getPoseFromTF(getLocalMapFrame());

        double odom_x = positionFromTF.x;
        double odom_y = positionFromTF.y;

        double pose_from_origin_x = calculator->getCoordinateX(*gps);
        double pose_from_origin_y = calculator->getCoordinateY(*gps);

        diff_x = pose_from_origin_x - odom_x;
        diff_y = pose_from_origin_y - odom_y;

        ROS_WARN("improve tf pose from gps x:%f y:%f", diff_x, diff_y);

        broadcaster_mutex.lock();
        transform.setOrigin(tf::Vector3(diff_x, diff_y, 0));
        broadcaster_mutex.unlock();
    }

    void TfHandler::improveTfRotation(double angle) {

        tf::Quaternion q;
        q.setRPY(0, 0, angle);
        broadcaster_mutex.lock();
        transform.setRotation(q);
        broadcaster_mutex.unlock();
    }

    void TfHandler::tfBroadcaster(){

        //inicialize TF broadcaster
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);

     //   std::string rotated_frame = local_map_frame + "_rotated";

        ros::Rate rate(10);

        while (ros::ok()){
            broadcaster_mutex.lock();
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, local_map_frame));
            broadcaster_mutex.unlock();
            rate.sleep();
        }
    }

    std::string TfHandler::getMapFrame(){ return map_frame; }
    std::string TfHandler::getBaseLinkFrame(){ return base_link_frame; }
    std::string TfHandler::getLocalMapFrame(){ return local_map_frame; }

}