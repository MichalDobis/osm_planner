//
// Created by controller on 11/12/18.
//

#include <ros/ros.h>


#ifndef PROJECT_COORDINATES_CONVERTER_BASE_H
#define PROJECT_COORDINATES_CONVERTER_BASE_H

namespace osm_planner {

    namespace coordinates_converters {

        class CoordinatesConverterBase {

        public:

            CoordinatesConverterBase() : offset_(OFFSET) {}

            CoordinatesConverterBase(double bearing) : offset_(bearing) {}

            virtual double getDistance(double latitude1, double longitude1, double latitude2, double longitude2) = 0;

            virtual double getBearing(double latitude1, double longitude1, double latitude2, double longitude2) = 0;

            void setOrigin(double latitude, double longitude) {
                origin_.latitude = latitude;
                origin_.longitude = longitude;
            }

            void setOffset(double offset) {
                this->offset_ = offset;
            }

            template<class N1>
            void setOrigin(N1 node) {
                setOrigin(node.latitude, node.longitude);
            }

            template<class N1, class N2>
            double getDistance(N1 node1, N2 node2) {
                return getDistance(node1.latitude, node1.longitude, node2.latitude, node2.longitude);
            };

            template<class N1, class N2>
            double getDistance(N1 node) {
                return getDistance(origin_.latitude, origin_.longitude, node.latitude, node.longitude);
            };

            template<class N1, class N2>
            double getBearing(N1 node1, N2 node2) {
                return getBearing(node1.latitude, node1.longitude, node2.latitude, node2.longitude);
            };

            template<class N>
            double getBearing(N node) {
                return getBearing(origin_.latitude, origin_.longitude, node.latitude, node.longitude);
            };

            template<class N>
            double getCoordinateX(N node) {

                double dist = getDistance(origin_, node);
                double bearing = getBearing(origin_, node);
                return sin(bearing + offset_) * dist;
            };

            template<class N1, class N2>
            double getCoordinateX(N1 node1, N2 node2) {

                double dist = getDistance(node1, node2);
                double bearing = getBearing(node1, node2);
                return sin(bearing + OFFSET) * dist;
            };

            template<class N>
            double getCoordinateY(N node) {

                double dist = getDistance(origin_, node);
                double bearing = getBearing(origin_, node);
                return cos(bearing + offset_) * dist;
            };

            template<class N1, class N2>
            double getCoordinateY(N1 node1, N2 node2) {

                //Haversine old method
                double dist = getDistance(node1, node2);
                double bearing = getBearing(node1, node2);
                return cos(bearing + OFFSET) * dist;
            };

        protected:

            struct osm_node {
                double latitude;
                double longitude;
                double altitude;
                double angle;
            };
            osm_node origin_;
            double offset_;


        private:

            constexpr static double OFFSET = M_PI / 2.0;
           // double offset_;
        };
    }
}

#endif //PROJECT_COORDINATES_CONVERTER_BASE_H
