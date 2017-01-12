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

//todo
//1. zistit preco pada pri hladany cesty v nejakom pripade  (sad_janka_krala.osm)
//2. zaviest vstup (volanim servisu generovat trajektoriu)
//3. lokalizovat sa na mape - asi spravit subscribera, ktory bude citat spravu sensor_msgs/NavSatFix (premysliet,
// ci bude zistovat aktualnu polohu stale alebo iba pri requeste o novu trajektoriu )
//4. presne lokalizovat ciel na mape
//5. noda publikuje body [x,y] - suradnice osm uzlov (nodes) + treba ako posledny bod doplnit bod z ciela (predposledny bude posledny osm uzol)
//6. treba urcit pociatocny bod [0,0] - asi prva suradnica,ktoru precita
//7. zvazit, ci nebude treba pouzivat action server
//8. preplanovanie trajektorie a odstranenie uzlu cez, ktory sa neda ist
//9. prepocitat zemepisne suradnice na kartezke - zatial je tam len vzorec x = (lon2 - lon1) * 1000; lon1 je bod v 0

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_osm");
	ros::NodeHandle n;

	std::string file = "skuska.osm";

	n.getParam("filepath", file);

	ROS_ERROR("%s",file.c_str());
   // ROS_ERROR("start time");
	OsmParser osm(file);
    osm.publishPath();
    Dijkstra dijkstra(osm.getGraphOfVertex());
    osm.publishPath(dijkstra.findShortestPath(120,100));
    //ROS_ERROR("stop time");

    ros::spin();


return 0;}
