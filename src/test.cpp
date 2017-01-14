/*
 * test.cpp
 *
 *  Created on: 17.10.2016
 *      Author: michal
 */

#include <ros/ros.h>
#include <osm_planner/dijkstra.h>
#include <osm_planner/osm_parser.h>
#include <osm_planner/source_and_target.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>

//todo
//1. zistit preco pada pri hladany cesty v nejakom pripade  (sad_janka_krala.osm)
//2. lokalizovat sa na mape - asi spravit subscribera, ktory bude citat spravu sensor_msgs/NavSatFix (premysliet,
// ci bude zistovat aktualnu polohu stale alebo iba pri requeste o novu trajektoriu )
//3. presne lokalizovat ciel na mape
//4. noda publikuje body [x,y] - suradnice osm uzlov (nodes) + treba ako posledny bod doplnit bod z ciela (predposledny bude posledny osm uzol)
//5. treba urcit pociatocny bod [0,0] - asi prva suradnica,ktoru precita
//6. zvazit, ci nebude treba pouzivat action server
//7. preplanovanie trajektorie a odstranenie uzlu cez, ktory sa neda ist
//8. prepocitat zemepisne suradnice na kartezke - zatial je tam len vzorec x = (lon2 - lon1) * 1000; lon1 je bod v 0

class OsmPlanner{
public:
    OsmPlanner(std::string file) :
            osm(file),
            dijkstra(osm.getGraphOfVertex()),
            target(100), source(130){

        //init ros topics and services
        ros::NodeHandle n;

        replanning_sub = n.subscribe("replanning", 1000, &OsmPlanner::changeTarget, this);
        gps_position_sub = n.subscribe("fix", 1000, &OsmPlanner::changeSource, this);

        replanning_service = n.advertiseService("replanning", &OsmPlanner::replanning, this);

        osm.publishPath();

        osm.publishPath(dijkstra.findShortestPath(source, target));

      //  osm.publishPath(dijkstra.findShortestPath(120, target));

    }
private:

    OsmParser osm;
    Dijkstra dijkstra;

    int source;
    int target;

    /* Subscribers */
    ros::Subscriber replanning_sub;
    ros::Subscriber gps_position_sub;

    /* Services */
    ros::ServiceServer replanning_service;

    //todo prerobit na vlastny service (request NavSatFix targetu)
    bool replanning(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

        osm.publishPath(dijkstra.findShortestPath(source, target));

        return true;
    }

    //todo ked sa prerobi service tuto funkciu odstranit
    void changeTarget(const std_msgs::Int32::ConstPtr& msg) {

        target = msg->data;
        ROS_ERROR("change target %d", target);

        osm.publishPath(dijkstra.getSolution(target));
    }

    //todo prerobit na msg NavSatFix, zatial cita len node ID
    void changeSource(const std_msgs::Int32::ConstPtr& msg) {

        source = msg->data;

        ROS_ERROR("change source %d", source);
        //todo tato funkcia sa bude vykonvat len docasne, potom sa bude vykonavat v service replanning
        osm.publishPath(dijkstra.findShortestPath(source, target));
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