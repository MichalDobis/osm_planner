///
// Created by controller on 11/13/18.
//

#include "osm_planner/coordinates_converters/wgs_84_elipsoid.h"

namespace osm_planner {

    namespace coordinates_converters {

        WGS84Elipsoid::WGS84Elipsoid() : CoordinatesConverterBase() {}

        WGS84Elipsoid::WGS84Elipsoid(double bearing) : CoordinatesConverterBase(bearing) {}

        double WGS84Elipsoid::getDistance(double latitude1, double longitude1, double latitude2, double longitude2) {

            geometry_msgs::Point p1 = getGeoPoint(latitude1, longitude1);
            geometry_msgs::Point p2 = getGeoPoint(latitude2, longitude2);
            return sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0));
        }

        double WGS84Elipsoid::getBearing(double latitude1, double longitude1, double latitude2, double longitude2) {

            geometry_msgs::Point p1 = getGeoPoint(latitude1, longitude1);
            geometry_msgs::Point p2 = getGeoPoint(latitude2, longitude2);

            return atan2( p1.y - p2.y, p1.x - p2.x);
        }

        geometry_msgs::Point WGS84Elipsoid::getGeoPoint(double latitude, double longitude){

            geometry_msgs::Point point;
            double N = a/(sqrt(1 - pow(e, 2.0)*pow(sin(latitude), 2.0)));

            point.x = (N + 0) * cos(latitude) * cos(longitude);
            point.y = (N + 0) * cos(latitude) * sin(longitude);
            point.z = (N * (1 - pow(e, 2.0) ) + 0) * sin(latitude);

            return point;
        }

    }
}