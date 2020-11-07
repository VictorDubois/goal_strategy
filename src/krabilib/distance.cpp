#include "krabilib/distance.h"

#include <math.h>

// On vérifie qu'il y a égalité ou presque entre deux distances
bool DistanceTools::distancePresqueEgales(Distance d1, Distance d2)
{
    static const Distance epsilon = 10;
    return (fabs(d1 - d2) < epsilon);
}

