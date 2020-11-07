#include "krabilib/vec3d.h"
#include "krabilib/positionPlusAngle.h"

Vec3d::Vec3d() : x(0), y(0), z(0)
{}

Vec3d::Vec3d(float x, float y, float z) : x(x), y(y), z(z)
{}

Vec3d Vec3d::operator+(const Vec3d &vec3d) const
{
    Vec3d resultat(x+vec3d.x,y+vec3d.y, z+vec3d.z);
    return resultat;
}

Vec3d Vec3d::operator-(const Vec3d &vec3d) const
{
    Vec3d resultat(x-vec3d.x,y-vec3d.y, z-vec3d.z);
    return resultat;
}

/// @brief Surchage d'opérateur pour multiplier par un flottant
Vec3d Vec3d::operator*(float val) const
{
    Vec3d resultat(x * val,y * val, z * val);
    return resultat;
}

void Vec3d::operator=(Vec3d vec3d)
{

    x = vec3d.x;
    y = vec3d.y;
    z = vec3d.z;

}

Vec3d Vec3d::operator+=(const Vec3d &vec3d)
{
    this->x += vec3d.x;
    this->y += vec3d.y;
    this->z += vec3d.z;

    return *this;
}

Vec3d Vec3d::operator-=(const Vec3d &vec3d)
{
    this->x -= vec3d.x;
    this->y -= vec3d.y;
    this->z += vec3d.z;

    return *this;
}

PositionPlusAngle Vec3d::operator+(const PositionPlusAngle &position) const
{
    PositionPlusAngle resultat(Position(x+position.getPosition().getX(), y+position.getPosition().getY()), z+position.getAngle());
    return resultat;
}

PositionPlusAngle Vec3d::operator-(const PositionPlusAngle &position) const
{
    PositionPlusAngle resultat(Position(x-position.getPosition().getX(), y-position.getPosition().getY()), z-position.getAngle());
    return resultat;
}
