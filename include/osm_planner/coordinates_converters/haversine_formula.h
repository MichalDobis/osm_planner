//
// Created by controller on 11/12/18.
//

#ifndef PROJECT_HAVERSINE_FORMULA_H
#define PROJECT_HAVERSINE_FORMULA_H

#include "osm_planner/coordinates_converters/coordinates_converter_base.h"

namespace osm_planner {

    namespace coordinates_converters {

//Embedded class for calculating distance and bearing
//Functions was inspired by: http://www.movable-type.co.uk/scripts/latlong.html
        class HaversineFormula : public CoordinatesConverterBase {

        public:

            HaversineFormula();
            HaversineFormula(double bearing);

            double getDistance(double latitude1, double longitude1, double latitude2, double longitude2) override;
            double getBearing(double latitude1, double longitude1, double latitude2, double longitude2) override;
        private:

            constexpr static double R = 6371e3;
            constexpr static double DEG2RAD = M_PI / 180;
            constexpr static double RAD2DEG = 180 / M_PI;
            constexpr static double OFFSET = M_PI / 2;

        };
    }
}

#endif //PROJECT_HAVERSINE_FORMULA_H
