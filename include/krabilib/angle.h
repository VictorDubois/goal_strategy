#pragma once
#include <boost/serialization/strong_typedef.hpp>

BOOST_STRONG_TYPEDEF(double, Angle);
BOOST_STRONG_TYPEDEF(double, AngleDeg);
BOOST_STRONG_TYPEDEF(double, AccelerationAngulaire);



class AngleTools
{
public:
    /// @brief Retourne la valeur de l'angle dans un float entre -PI et PI.
    static Angle wrapAngle(Angle angle);

    /// @brief Vérifie s'il y a égalité ou presque entre deux angles.
    static bool anglesAlmostEqual(Angle a1, Angle a2, Angle eplsion = Angle(0.1));

    /// @brief Calcul la différence entre deux angles (rad), le résultat est donné dans [-pi, pi]
    static Angle diffAngle(Angle a, Angle b);

    /**
     * @brief Convert from radian to degrees
     * 
     * @param a angle to convert in degree
     * @return Angle in radian
     */
    static AngleDeg rad2deg(Angle a);

    /**
     * @brief Convert from degrees to radian
     * 
     * @param a angle to convert in radian
     * @return AngleDeg in degree
     */
    static Angle deg2rad(AngleDeg a);
};


