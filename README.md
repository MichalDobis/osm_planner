# ROS OpenStreetMap Planner
Experimental ROS package for finding path on OpenStreetMap using dijkstra algorithm

##Subscribed Topic
*/fix ([sensor_msgs/NavSatFix](http://docs.ros.org/jade/api/sensor_msgs/html/msg/NavSatFix.html))* - receiving your geographics position. Current position is published as marker

##Published Topic
*/position_marker ([visualization_msgs/Marker](http://docs.ros.org/jade/api/visualization_msgs/html/msg/Marker.html))* - publishing your current position<br>
*/target_marker ([visualization_msgs/Marker](http://docs.ros.org/jade/api/visualization_msgs/html/msg/Marker.html))* - publishing your current target<br>
*/obstacle_marker ([visualization_msgs/Marker](http://docs.ros.org/jade/api/visualization_msgs/html/msg/Marker.html))* - publishing localizated obstacles on the road. Obstacle can be localized by */cancel_point* service<br>
*/path ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))* - publishing network of all footways<br>
*/shortest_path ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))* - publishing found shortest path<br>

##Services
*/replanning ([osm_planner/newTarget](https://github.com/MichalDobis/osm_planner/blob/master/srv/newTarget.srv))* - execute to find shortest path from current position to selected target<br>
*/cancel_point ([osm_planner/cancelledPoint](https://github.com/MichalDobis/osm_planner/blob/master/srv/cancelledPoint.srv))* - delete edge of graph, where is any obstacle and trajectory must be replane. The cancelled point is index of array (published shortest path). This index is first osm node id and index + 1 is second osm node id. Deleted edge will lie between these osm nodes.<br>
