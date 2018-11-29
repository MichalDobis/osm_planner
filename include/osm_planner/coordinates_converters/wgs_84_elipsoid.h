//
// Created by controller on 11/13/18.
//

#ifndef PROJECT_WGS_84_ELIPSOID_H
#define PROJECT_WGS_84_ELIPSOID_H

#include <geometry_msgs/Point.h>
#include "osm_planner/coordinates_converters/coordinates_converter_base.h"

namespace osm_planner {

    namespace coordinates_converters {

//Embedded class for calculating distance and bearing
//Functions was inspired by: http://www.movable-type.co.uk/scripts/latlong.html
        class WGS84Elipsoid : public CoordinatesConverterBase {

        public:

            WGS84Elipsoid();
            WGS84Elipsoid(double bearing);

            double getDistance(double latitude1, double longitude1, double latitude2, double longitude2) override;
            double getBearing(double latitude1, double longitude1, double latitude2, double longitude2) override;
        private:

            geometry_msgs::Point point_;

            constexpr static double a = 6375137; //m - dlzka hlavnej polosi, polomer rovnika
            constexpr static double b = 6356753; //m - dlzka vedlajsej polosi, polomer poludnika
            constexpr static double f = (a - b)/a; //spolostenie elipsoidu Zeme
            constexpr static double e = 2*f - f*f; //prva excentricita

            geometry_msgs::Point getGeoPoint(double latitude, double longitude);

        };
    }
}

#endif //PROJECT_WGS_84_ELIPSOID_H
