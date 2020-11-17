#include "krabilib/distance.h"

#include <math.h>

Distance operator* (Distance d, float k){
    return Distance(float(d)*k);
}

Distance operator* (Distance d, double k){
    return Distance(double(d)*k);
}

Vitesse operator* (Vitesse d, float k){
    return Vitesse(float(d)*k);
}

Vitesse operator* (Vitesse d, double k){
    return Vitesse(double(d)*k);
}

Acceleration operator* (Acceleration d, float k){
    return Acceleration(float(d)*k);
}

Acceleration operator* (Acceleration d, double k){
    return Acceleration(double(d)*k);
}

// On vérifie qu'il y a égalité ou presque entre deux distances
bool DistanceTools::distancePresqueEgales(Distance d1, Distance d2, Distance epsilon)
{
    return (fabs(d1 - d2) < epsilon);
}

