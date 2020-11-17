#include "krabilib/angle.h"
#include <math.h>

// Retourne la valeur de l'angle dans un float entre -PI et PI.
Angle AngleTools::wrapAngle(Angle angle)
{

    if (angle > 0)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
    }
    else
    {
        while (angle < -M_PI)
            angle += 2 * M_PI;
    }
    return angle;
}

// Vérifie s'il y a égalité ou presque entre deux angles.
bool AngleTools::anglesAlmostEqual(Angle a1, Angle a2, Angle epsilon)
{
    return (fabs(diffAngle(a1, a2)) < epsilon);
}

// Calcul la différence entre deux angles (rad)
// le résultat est dans [-pi, pi]
Angle AngleTools::diffAngle(Angle a, Angle b)
{
    Angle t(a - b);
    while (t > M_PI)
    {
        t -= 2 * M_PI;
    }
    while (t < -M_PI)
    {
        t += 2 * M_PI;
    }
    return t;
}

AngleDeg AngleTools::rad2deg(Angle a)
{
    return AngleDeg(double(a) * 180 / M_PI);
}

Angle AngleTools::deg2rad(AngleDeg a)
{
    return Angle(double(a) * M_PI / 180);
}

Angle operator*(Angle d, float k)
{
    return Angle(float(d) * k);
}

Angle operator*(Angle d, double k)
{
    return Angle(double(d) * k);
}

VitesseAngulaire operator*(VitesseAngulaire d, float k)
{
    return VitesseAngulaire(float(d) * k);
}

VitesseAngulaire operator*(VitesseAngulaire d, double k)
{
    return VitesseAngulaire(double(d) * k);
}

AngleDeg operator*(AngleDeg d, float k)
{
    return AngleDeg(float(d) * k);
}

AngleDeg operator*(AngleDeg d, double k)
{
    return AngleDeg(double(d) * k);
}

Angle operator/(Angle d, float k)
{
    return Angle(float(d) / k);
}

Angle operator/(Angle d, double k)
{
    return Angle(double(d) / k);
}

VitesseAngulaire operator/(VitesseAngulaire d, float k)
{
    return VitesseAngulaire(float(d) / k);
}

VitesseAngulaire operator/(VitesseAngulaire d, double k)
{
    return VitesseAngulaire(double(d) / k);
}

AngleDeg operator/(AngleDeg d, float k)
{
    return AngleDeg(float(d) / k);
}

AngleDeg operator/(AngleDeg d, double k)
{
    return AngleDeg(double(d) / k);
}

Angle operator-(Angle d1, Angle d2)
{
    return Angle(double(d1) - d2);
}

VitesseAngulaire operator-(VitesseAngulaire d1, VitesseAngulaire d2)
{
    return VitesseAngulaire(double(d1) - d2);
}

AngleDeg operator-(AngleDeg d1, AngleDeg d2)
{
    return AngleDeg(double(d1) - d2);
}
