#ifndef VEC3D_H
#define VEC3D_H

class PositionPlusAngle;

class Vec3d
{
public:
    Vec3d();
    Vec3d(float x, float y, float z);

    float x, y, z;

    Vec3d operator+(const Vec3d &vec3d) const;

    Vec3d operator-(const Vec3d &vec3d) const;

    /// @brief Surchage d'opérateur pour multiplier par un flottant
    Vec3d operator*(float val) const;

    void operator=(Vec3d vec3d);

    Vec3d operator+=(const Vec3d &Vec3d);

    Vec3d operator-=(const Vec3d &vec3d);

    PositionPlusAngle operator+(const PositionPlusAngle &position) const;

    PositionPlusAngle operator-(const PositionPlusAngle &position) const;
};

#endif // VEC3D_H
