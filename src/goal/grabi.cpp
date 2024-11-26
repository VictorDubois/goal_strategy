#include <goal_strategy/grabi.h>

Grabi::Grabi(Position a_relative_position_in_front,
             Position a_relative_position_inside,
             std::shared_ptr<Servomotor> a_servo_magnet_1,
             std::shared_ptr<Servomotor> a_servo_magnet_2,
             std::shared_ptr<Servomotor> a_servo_magnet_3,
             std::shared_ptr<Servomotor> a_servo_magnet_4,
             std::shared_ptr<StepperElevator> a_stepper_elevator)
  : m_relative_position_in_front(a_relative_position_in_front)
  , m_relative_position_inside(a_relative_position_inside)
  , m_relative_position(a_relative_position_in_front)
  , m_servo_magnet_1(a_servo_magnet_1)
  , m_servo_magnet_2(a_servo_magnet_2)
  , m_servo_magnet_3(a_servo_magnet_3)
  , m_servo_magnet_4(a_servo_magnet_4)
  , m_stepper_elevator(a_stepper_elevator)
{
}

void Grabi::setInFront()
{
    m_relative_position = m_relative_position_in_front;
}
void Grabi::setInside()
{
    m_relative_position = m_relative_position_inside;
}

/*
 * Returns the reach of Grabi, with respect to the center of the robot
 */
float Grabi::getReach()
{
    return m_relative_position.getNorme();
}

Angle Grabi::getAngle()
{
    return m_relative_position.getAngle();
}

void Grabi::grab_plateforme(bool a_do_sleep)
{
    m_right_servo->set(150, 100);

    // Pour ne pas s'emmeler les pinces
    if (m_left_servo->getAngle() < 70)
    {
        conditionnal_sleep(1e6, a_do_sleep);
    }
    m_left_servo->set(s_angle_closed_left, 100);
    conditionnal_sleep(1.5e6, a_do_sleep);
}

void Grabi::drop_plateforme(bool a_do_sleep)
{
    m_right_servo->set(90, 100);

    // Pour ne pas s'emmeler les pinces
    if (m_left_servo->getAngle() < s_angle_closed_left)
    {
        conditionnal_sleep(1e6, a_do_sleep);
    }
    m_left_servo->set(125, 100);
    conditionnal_sleep(0.75e6, a_do_sleep);
}


void Grabi::conditionnal_sleep(uint a_microseconds, bool a_do_sleep)
{
    if (a_do_sleep)
    {
        usleep(a_microseconds);
    }
}
