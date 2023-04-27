#include <goal_strategy/claws.h>

Claws::Claws(Position a_relative_position_in_front,
             Position a_relative_position_inside,
             std::shared_ptr<Servomotor> a_left_servo,
             std::shared_ptr<Servomotor> a_right_servo)
  : m_relative_position_in_front(a_relative_position_in_front)
  , m_relative_position_inside(a_relative_position_inside)
  , m_relative_position(a_relative_position_in_front)
  , m_left_servo(a_left_servo)
  , m_right_servo(a_right_servo)
{
}

void Claws::setInFront()
{
    m_relative_position = m_relative_position_in_front;
}
void Claws::setInside()
{
    m_relative_position = m_relative_position_inside;
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

void Claws::grab_pile(bool a_do_sleep)
{
    m_right_servo->set(130, 10);

    // Pour ne pas s'emmeler les pinces
    if (m_left_servo->getAngle() < 70)
    {
        conditionnal_sleep(1e6, a_do_sleep);
    }
    m_left_servo->set(80, 10);
    conditionnal_sleep(1e6, a_do_sleep);
}

void Claws::release_pile(bool a_do_sleep)
{
    m_right_servo->set(90, 10);

    // Pour ne pas s'emmeler les pinces
    if (m_left_servo->getAngle() < 70)
    {
        conditionnal_sleep(1e6, a_do_sleep);
    }
    m_left_servo->set(120, 10);
    conditionnal_sleep(1e6, a_do_sleep);
}

void Claws::retract(bool a_do_sleep)
{
    // Do nothing, retracting is dangerous if there is a pile stuck :/
    grab_pile(a_do_sleep);
    /*m_left_servo->set(16, 20);
    usleep(1e6); // Pour ne pas s'emmeler les pinces

    m_right_servo->set(180, 20);
    usleep(1e6);*/
}

void Claws::conditionnal_sleep(uint a_microseconds, bool a_do_sleep)
{
    if (a_do_sleep)
    {
        usleep(a_microseconds);
    }
}
