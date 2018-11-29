//
// Created by controller on 11/13/18.
//

#include "osm_planner/coordinates_converters/haversine_formula.h"

namespace osm_planner {

    namespace coordinates_converters {

        HaversineFormula::HaversineFormula() : CoordinatesConverterBase() {}

        HaversineFormula::HaversineFormula(double bearing) : CoordinatesConverterBase(bearing) {}

        double HaversineFormula::getDistance(double latitude1, double longitude1, double latitude2, double longitude2) {

            /*  Haversine formula:
            *  a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
            *  c = 2 ⋅ atan2( √a, √(1−a) )
            *  d = R ⋅ c
            *
            *  φ - latitude;
            *  λ - longitude;
            */
            double dLat = latitude2 * DEG2RAD - latitude1 * DEG2RAD;
            double dLon = longitude2 * DEG2RAD - longitude1 * DEG2RAD;
            double a = sin(dLat / 2) * sin(dLat / 2) +
                       cos(latitude1 * DEG2RAD) * cos(latitude2 * DEG2RAD) *
                       sin(dLon / 2) * sin(dLon / 2);
            double c = 2 * atan2(sqrt(a), sqrt(1 - a));
            return R * c;
        }

        double HaversineFormula::getBearing(double latitude1, double longitude1, double latitude2, double longitude2) {

            /*   Haversine formula:
            *   a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
            *   c = 2 ⋅ atan2( √a, √(1−a) )
            *   d = R ⋅ c
            *
            *  φ - latitude;
            *  λ - longitude;
            */
            double dLon = longitude2 * DEG2RAD - longitude1 * DEG2RAD;
            double y = sin(dLon) * cos(latitude2 * DEG2RAD);
            double x = cos(latitude1 * DEG2RAD) * sin(latitude2 * DEG2RAD) -
                       sin(latitude1 * DEG2RAD) * cos(latitude2 * DEG2RAD) * cos(dLon);
            return atan2(y, x) + offset_;
        }
    }
}