#ifndef VEC2D_H
#define VEC2D_H

#include "Krabi/angle.h"

class Position;

class Vec2d
{
public:
    Vec2d();
    Vec2d(float x, float y);

    float x, y;

    Vec2d operator+(const Vec2d &vec2d) const;

    Vec2d operator-(const Vec2d &vec2d) const;

    /// @brief Surchage d'opérateur pour multiplier par un flottant
    Vec2d operator*(float val) const;

    void operator=(Vec2d vec2d);

    Vec2d operator+=(const Vec2d &vec2d);

    Vec2d operator-=(const Vec2d &vec2d);

    Position operator+(const Position &position) const;

    Position operator-(const Position &position) const;

    float getX() const;

    float getY() const;

    float getNorme() const;

    Angle getAngle(void) const;
};

#endif // VEC2D_H
