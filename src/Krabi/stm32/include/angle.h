#ifndef ANGLE_H
#define ANGLE_H

/// @brief On définie un Angle comme étant un float pour que l'on comprenne mieux le programme.
typedef float Angle;
typedef float VitesseAngulaire;
typedef float AccelerationAngulaire;

enum Sign
{
    SGN_NEG = -1,
    SGN_UNDEF = 0,
    SGN_POS = 1
};

class AngleTools
{
public:
    /// @brief Retourne la valeur de l'angle dans un float entre -PI et PI.
    static Angle wrapAngle(Angle angle);

    /// @brief Vérifie s'il y a égalité ou presque entre deux angles.
    static bool anglesAlmostEqual(Angle a1, Angle a2);

    /// @brief Calcul la différence entre deux angles (rad), le résultat est donné dans [-pi, pi]
    static float diffAngle(float a, float b);
};

#endif // ANGLE_H

