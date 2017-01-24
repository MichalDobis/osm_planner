# osm_planner
Experimental ROS package for finding path on OpenStreetMap using dijkstra algorithm

##Subscribed Topic
*/fix ([sensor_msgs/NavSatFix](http://docs.ros.org/jade/api/sensor_msgs/html/msg/NavSatFix.html))* - receiving your geographics position. Current position is published as marker

##Published Topic
*/visualization_marker ([visualization_msgs/Marker](http://docs.ros.org/jade/api/visualization_msgs/html/msg/Marker.html))* - publishing your current position and target<br>
*/path ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))* - publishing network of all footways<br>
*/shortest_path ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))* - publishing found shortest path<br>

##Services
*/replanning ([osm_planner/newTarget](https://github.com/MichalDobis/osm_planner/blob/master/srv/newTarget.srv))* - execute to find shortest path from current position to selected target
