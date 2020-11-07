#include <math.h>

#include "krabilib/vec2d.h"

#include "krabilib/position.h"

Vec2d::Vec2d() : x(0), y(0)
{}

Vec2d::Vec2d(float x, float y) : x(x), y(y)
{}

Vec2d Vec2d::operator+(const Vec2d &vec2d) const
{
    Vec2d resultat(x+vec2d.x,y+vec2d.y);
    return resultat;
}

Vec2d Vec2d::operator-(const Vec2d &vec2d) const
{
    Vec2d resultat(x-vec2d.x,y-vec2d.y);
    return resultat;
}

/// @brief Surchage d'opérateur pour multiplier par un flottant
Vec2d Vec2d::operator*(float val) const
{
    Vec2d resultat(x * val,y * val);
    return resultat;
}

void Vec2d::operator=(Vec2d vec2d)
{

    x = vec2d.x;
    y = vec2d.y;

}

Vec2d Vec2d::operator+=(const Vec2d &vec2d)
{
    this->x += vec2d.x;
    this->y += vec2d.y;

    return *this;
}

Vec2d Vec2d::operator-=(const Vec2d &vec2d)
{
    this->x -= vec2d.x;
    this->y -= vec2d.y;

    return *this;
}

Position Vec2d::operator+(const Position &position) const
{
    Position resultat(x+position.x, y+position.y, true);
    return resultat;
}

Position Vec2d::operator-(const Position &position) const
{
    Position resultat(x-position.x, y-position.y, true);
    return resultat;
}

float Vec2d::getX() const
{
    return x;
}

float Vec2d::getY() const
{
    return y;
}

Distance Vec2d::getNorme() const
{
    return Distance(sqrt(x*x+y*y));
}

Angle Vec2d::getAngle() const
{
    return atan2(y,x);
}
