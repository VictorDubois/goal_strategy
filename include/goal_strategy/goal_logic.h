#pragma once

/**
 * Pure, ROS-free geometry/logic helpers extracted from GoalStrat (main.cpp).
 *
 * Everything here takes plain inputs and returns plain outputs: no rclcpp node,
 * no TF lookups, no member state, and crucially NO year #ifdef. That makes these
 * functions unit-testable in isolation, and the assertions about them stay valid
 * year after year (see test/test_goal_logic.cpp).
 *
 * Year-specific values (reach distances per actuator, park-spot coordinates,
 * the 2024 direction flip, ...) are deliberately kept OUT of these functions and
 * passed in as parameters, or left in the GoalStrat node wrappers that call them.
 */

#include <vector>

#include "krabilib/angle.h"
#include "krabilib/distance.h"
#include "krabilib/position.h"
#include "krabilib/strategie/etape.h"

namespace goal_logic
{

/**
 * @brief Has the robot reached @p target ?
 * @param cur    current robot position
 * @param target position to reach
 * @param reach  arrival threshold (caller decides the value; may be year-specific)
 * @return true when the centre-to-centre distance is strictly below @p reach
 */
bool isWithinReach(const Position& cur, const Position& target, Distance reach);

/**
 * @brief Is the robot's tool aligned with @p target angle ?
 * @param target      desired absolute angle
 * @param baseAngle   current robot heading (before any reverse-gear correction)
 * @param reverseGear true when the active tool is on the back of the robot: the
 *                    effective tool angle is then @p baseAngle + pi
 * @param reachAng    angular tolerance
 * @return true when the wrapped angular error is strictly below @p reachAng
 */
bool isAlignedWithAngle(Angle target, Angle baseAngle, bool reverseGear, Angle reachAng);

/**
 * @brief Point @p dist away from @p cur along @p heading, used for straight moves.
 * @param sensRecule true to move backward (subtract), false to move forward (add)
 * @return the target position; its distance to @p cur is exactly @p dist
 */
Position straightLineTarget(const Position& cur, Angle heading, Distance dist, bool sensRecule);

/**
 * @brief Max speed allowed at arrival for the given step type.
 * @return 0.1 for a pass-through point (no need to stop), 0 otherwise (brake).
 */
float maxSpeedAtArrival(Etape::EtapeType type);

/**
 * @brief Is the robot within @p radius of any of the valid parking @p spots ?
 * @param spots valid end locations (caller supplies the year-specific coordinates)
 */
bool isParkedAt(const Position& cur, const std::vector<Position>& spots, Distance radius);

} // namespace goal_logic
