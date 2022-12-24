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
    m_right_servo->set(140, 3);

    // Pour ne pas s'emmeler les pinces
    if (m_left_servo->getAngle() < 70)
    {
        usleep(1e6);
    }
    m_left_servo->set(70, 3);
    usleep(1e6);
}

void Claws::release_pile()
{
    m_right_servo->set(90, 3);

    // Pour ne pas s'emmeler les pinces
    if (m_left_servo->getAngle() < 70)
    {
        usleep(1e6);
    }
    m_left_servo->set(120, 3);
    usleep(1e6);
}

void Claws::retract()
{
    m_left_servo->set(16, 3);
    usleep(1e6); // Pour ne pas s'emmeler les pinces

    m_right_servo->set(180, 3);
    usleep(1e6);
}
