#ifndef DISTANCE_H
#define DISTANCE_H

#ifdef ROBOTHW
    #define abs(x) x > 0 ? x : -x
#endif

/// @brief On définie Distance comme un float pour une meilleur lisibilité
typedef float Distance;

/// @brief On définie Vitesse comme un float pour une meilleur lisibilité
typedef float Vitesse;

/// @brief On définie Acceleration comme un float pour une meilleur lisibilité
typedef float Acceleration;

class DistanceTools
{
public:
    /// @brief On vérifie qu'il y a égalité ou presque entre deux distances
    static bool distancePresqueEgales(Distance d1, Distance d2);
};

#endif // DISTANCE_H
