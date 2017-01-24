/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

#include <ros/ros.h>
#include <osm_planner/dijkstra.h>
#include <osm_planner/osm_parser.h>
#include <osm_planner/newTarget.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>

//todo
//1. zistit preco pada pri hladany cesty v nejakom pripade  (sad_janka_krala.osm)
//2. pretestovat funkciu OsmParser::getNearestPoint()
//3. noda publikuje body [x,y] - suradnice osm uzlov (nodes) + treba ako posledny bod doplnit bod z ciela (predposledny bude posledny osm uzol)
//4. treba urcit pociatocny bod [0,0] - asi prva suradnica, ktoru precita.
//   Do gpsCallback treba pridat nejaku funkciu, ktora urci pociatocny bod z prvej suradnice a ulozi ju do globalnej premennej OsmParser
//5. zvazit, ci nebude treba pouzivat action server
//6. preplanovanie trajektorie a odstranenie uzlu cez, ktory sa neda ist
//7. prepocitat zemepisne suradnice na kartezke - zatial je tam len vzorec x = (lon2 - lon1) * 1000; lon1 je bod v 0

class OsmPlanner{
public:
    OsmPlanner(std::string file) :
            osm(file),
            dijkstra(osm.getGraphOfVertex()),
            targetID(100), sourceID(130){

        //init ros topics and services
        ros::NodeHandle n;

        replanning_sub = n.subscribe("replanning", 1, &OsmPlanner::changeTarget, this);
        gps_position_sub = n.subscribe("fix", 1, &OsmPlanner::gpsCallback, this);

        change_source_sub = n.subscribe("change_source", 1, &OsmPlanner::changeSource, this);

        replanning_service = n.advertiseService("replanning", &OsmPlanner::replanning, this);

        osm.publishPath();

        osm.publishPath(dijkstra.findShortestPath(sourceID, targetID));

        colorPosition.r = 1.0f;
        colorPosition.g = 1.0f;
        colorPosition.b = 0.0f;
        colorPosition.a = 1.0;

        colorTarget.r = 1.0f;
        colorTarget.g = 0.0f;
        colorTarget.b = 0.0f;
        colorTarget.a = 1.0;

        osm.publishPoint(sourceID, colorPosition);

        sleep(1);
        osm.publishPoint(targetID, colorTarget);

        //  osm.publishPath(dijkstra.findShortestPath(120, target));

    }
private:

    OsmParser osm;
    Dijkstra dijkstra;

    int sourceID;
    int targetID;

    visualization_msgs::Marker::_color_type colorPosition;
    visualization_msgs::Marker::_color_type colorTarget;

    /* Subscribers */
    ros::Subscriber replanning_sub;
    ros::Subscriber gps_position_sub;
    ros::Subscriber change_source_sub;

    /* Services */
    ros::ServiceServer replanning_service;

    //todo prerobit na vlastny service (request NavSatFix targetu)
    bool replanning(osm_planner::newTarget::Request &req, osm_planner::newTarget::Response &res){

        targetID = osm.getNearestPoint(req.target.latitude, req.target.longitude);
        osm.publishPath(dijkstra.findShortestPath(sourceID, targetID));

        //todo dorobit v pripade, ze sa nenaplanuje, tak res.success = false
        res.success = true;
        return true;
    }

    //todo ked sa prerobi service tuto funkciu odstranit
    void changeTarget(const std_msgs::Int32::ConstPtr& msg) {

        targetID = msg->data;
        ROS_ERROR("change target %d", targetID);

        osm.publishPath(dijkstra.getSolution(targetID));
        osm.publishPoint(targetID, colorTarget);

    }

    //len pre test, bude sa pouzivat gpsCallback
    void changeSource(const std_msgs::Int32::ConstPtr& msg) {

        sourceID = msg->data;

        ROS_ERROR("change source %d", sourceID);
        //todo tato funkcia sa bude vykonvat len docasne, potom sa bude vykonavat v service replanning
        osm.publishPath(dijkstra.findShortestPath(sourceID, targetID));
        osm.publishPoint(sourceID, colorPosition);

    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {


        double lat = msg->latitude;
        double lon = msg->longitude;

        osm.publishPoint(lat, lon, colorTarget);
        sourceID = osm.getNearestPoint(lat, lon);

        ROS_ERROR(" new source %d", sourceID);

        osm.publishPoint(sourceID, colorPosition);
    }
};


int main(int argc, char **argv) {

	ros::init(argc, argv, "test_osm");
	ros::NodeHandle n;

	std::string file = "skuska.osm";
	n.getParam("filepath", file);
    OsmPlanner osm_planner(file);

    ros::spin();

return 0;
}