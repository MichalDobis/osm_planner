//
// Created by michal on 31.12.2016.
// Modify and refactor by michal on 11.11.2018
//

#ifndef OSM_LOCALIZATION_H
#define OSM_LOCALIZATION_H

#include <osm_planner/osm_parser.h>
#include <geometry_msgs/Pose.h>

namespace osm_planner {

    //! OSM Localization class
/*!
 * The localization on the osm map.
 * It's create a union of geographic (GPS) pose, cartesian (XY) pose and nearest point on the map
*/
    class Localization {
    public:

        /**
         * @brief Set pointer to map class and set default values
         * @param map - A pointer to map. The map should be constructed and parsed
         * @param name - Optional param, just for print message
         */
        Localization(std::shared_ptr<osm_planner::Parser> map, std::string name = "");

        /**
         * @brief Setter Footway Width
         * @param footway_width - It's used in getDistanceFromWayCenter()
         */
        void setFootwayWidth(const double footway_width);

        /**
         * @brief Set position from GPS data, then calculate nearest map node
         * and cartesian position from map origin
         * @param lat - latitude
         * @param lon - longitude
         */
        void setPositionFromGPS(const double lat, const double lon);

        /**
         * @brief Set position from pose, then calculate nearest map node
         * and set geogpraphic position from map node
         * @param pose - cartesian position
         */
        void setPositionFromPose(const geometry_msgs::Pose &pose);  //from odom

        /**
         * @brief Getter Footway Width
         * @return Width of footway
         */
        double getFootwayWidth();

        /**
         * @brief is position from gps
         * @return A True if is last position set by setPositionFromGPS()
         */
        bool isPositionFromGps();

        /**
         * @brief Get ID from nearest node on the map
         * @return A ID from nearest node on the map
         */
        int getPositionNodeID();

        /**
         * @brief Get cartesian pose
         * @return A cartesian pose
         */
        geometry_msgs::Pose getPose();
        /**
         * @brief Get geographic position
         * @return A geographic position
         */
        Parser::OSM_NODE getGeoPoint();

        /**
         * @brief Get distance from way calculated by formula:
         * Distance between last position (GPS/cartesian) - footway width
         * @return A distance from way
         */
        double getDistanceFromWay();

    private:

        /**
         * @brief All representations of position
         * @var id - ID of nearest node
         * @var geo_point - geographic coordinate
         * @var cartesian_pose - cartesian coordinate
         * @var is_set_from_gps - True if is last pose from setPositionFromGPS()
         */
        struct positionsData{
            int id;
            Parser::OSM_NODE geo_point;
            geometry_msgs::Pose cartesian_pose;
            bool is_set_from_gps;
        };

        positionsData positions_;                       ///< All representations of position
        std::shared_ptr<osm_planner::Parser> map_;      ///< A pointer to map
        double footway_width_;                          ///< Footway witdh
        std::string name_;                              ///< Just for print message
    };
}

#endif //OSM_LOCALIZATION_H
