#include <goal_strategy/claws.h>

Claws::Claws(Position a_relative_position,
             std::shared_ptr<Servomotor> a_left_servo,
             std::shared_ptr<Servomotor> a_right_servo)
  : m_relative_position(a_relative_position)
  , m_left_servo(a_left_servo)
  , m_right_servo(a_right_servo)
{
}

/*
 * Returns the reach of the claws, with respect to the center of the robot
 */
float Claws::getReach()
{
    return m_relative_position.getNorme();
}

Angle Claws::getAngle()
{
    return m_relative_position.getAngle();
}

void Claws::grab_pile()
{
    m_left_servo->setAngle(16);
    m_right_servo->setAngle(36);

    m_left_servo->setSpeed(3);
    m_right_servo->setSpeed(3);
    usleep(1000000);
}

void Claws::release_pile()
{
    m_left_servo->setAngle(16);
    m_right_servo->setAngle(36);

    m_left_servo->setSpeed(3);
    m_right_servo->setSpeed(3);
    usleep(1000000);
}

void Claws::retract()
{
    m_left_servo->setAngle(16);
    m_right_servo->setAngle(36);

    m_left_servo->setSpeed(3);
    m_right_servo->setSpeed(3);
    usleep(1000000);
}
