#pragma once

#include <boost/serialization/strong_typedef.hpp>

BOOST_STRONG_TYPEDEF(double, Distance);
BOOST_STRONG_TYPEDEF(double, Vitesse);
BOOST_STRONG_TYPEDEF(double, Acceleration);


class DistanceTools
{
public:
    /// @brief On vérifie qu'il y a égalité ou presque entre deux distances
    static bool distancePresqueEgales(Distance d1, Distance d2, Distance epsilon = Distance(0.005));
};

Distance operator* (Distance d, float k);
Distance operator* (Distance d, double k);
Vitesse operator* (Vitesse d, float k);
Vitesse operator* (Vitesse d, double k);
Acceleration operator* (Acceleration d, float k);
Acceleration operator* (Acceleration d, double k);
Distance operator/ (Distance d, float k);
Distance operator/ (Distance d, double k);
Vitesse operator/ (Vitesse d, float k);
Vitesse operator/ (Vitesse d, double k);
Acceleration operator/ (Acceleration d, float k);
Acceleration operator/ (Acceleration d, double k);