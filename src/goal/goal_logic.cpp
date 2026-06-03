#include "goal_strategy/goal_logic.h"

#include <cmath>

namespace goal_logic
{

bool isWithinReach(const Position& cur, const Position& target, Distance reach)
{
    return (cur - target).getNorme() < reach;
}

bool isAlignedWithAngle(Angle target, Angle baseAngle, bool reverseGear, Angle reachAng)
{
    // When the active tool is on the back of the robot, the heading we want to
    // align is the robot heading turned by 180 degrees.
    Angle toolAngle = baseAngle;
    if (reverseGear)
    {
        toolAngle = toolAngle + Angle(M_PI);
    }

    Angle angular_error = AngleTools::diffAngle(target, toolAngle);
    return std::abs(static_cast<double>(angular_error)) < static_cast<double>(reachAng);
}

Position straightLineTarget(const Position& cur, Angle heading, Distance dist, bool sensRecule)
{
    const double angle = heading;                // Angle -> double
    const double sign = sensRecule ? -1.0 : 1.0; // recule subtracts, avance adds

    Position result = cur;
    result.setX(Distance(cur.getX() + dist * sign * std::cos(angle)));
    result.setY(Distance(cur.getY() + dist * sign * std::sin(angle)));
    return result;
}

float maxSpeedAtArrival(Etape::EtapeType type)
{
    // No need for a complete stop at intermediate pass-through points.
    return (type == Etape::EtapeType::POINT_PASSAGE) ? 0.1f : 0.f;
}

bool isParkedAt(const Position& cur, const std::vector<Position>& spots, Distance radius)
{
    for (const auto& spot : spots)
    {
        if ((cur - spot).getNorme() < radius)
        {
            return true;
        }
    }
    return false;
}

} // namespace goal_logic
