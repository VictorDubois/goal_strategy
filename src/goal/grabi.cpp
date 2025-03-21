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

void Grabi::initializeElevator()
{
    m_stepper_elevator->doHoming();
}

bool Grabi::elevatorInitDone()
{
    return m_stepper_elevator->homingDone();
}

bool Grabi::grab_plateforme(bool a_do_sleep)
{
    if (elevatorInitHasFailed())
    {
        return false;
    }
    
    /* @todo raise plank */

    m_stepper_elevator->goToPosition(50); // mm
    conditionnal_sleep(1.5e6, a_do_sleep);

    m_servo_magnet_1->set(150, 100);
    m_servo_magnet_2->set(150, 100);
    m_servo_magnet_3->set(150, 100);
    m_servo_magnet_4->set(150, 100);
    conditionnal_sleep(1.5e6, a_do_sleep);
    m_stepper_elevator->goToPosition(150); // mm
    return true;
}

bool Grabi::elevatorInitHasFailed()
{
    rclcpp::Time timeout = rclcpp::Clock{RCL_STEADY_TIME}.now() + rclcpp::Duration(3, 0);
    
    while (rclcpp::Clock{RCL_STEADY_TIME}.now() < timeout)
    {
        initializeElevator();
        if (elevatorInitDone())
        {
            return true;
        }
        
    }
    return false;
}

bool Grabi::drop_plateforme(bool a_do_sleep)
{
    if (elevatorInitHasFailed())
    {
        return false;
    }
    
    m_stepper_elevator->goToPosition(50); // mm
    rclcpp::Time timeout = rclcpp::Clock{RCL_STEADY_TIME}.now() + rclcpp::Duration(3, 0);
    bool success = false;
    while(a_do_sleep && !success && rclcpp::Clock{RCL_STEADY_TIME}.now() < timeout)
    {
        usleep(10);
        if (m_stepper_elevator->movmentDone())
        {
            success = true;
        }
    }
    m_servo_magnet_1->set(50, 100);
    m_servo_magnet_2->set(50, 100);
    m_servo_magnet_3->set(50, 100);
    m_servo_magnet_4->set(50, 100);
    conditionnal_sleep(0.75e6, a_do_sleep);
    m_stepper_elevator->goToPosition(150); // mm
    return success;
}


void Grabi::conditionnal_sleep(uint a_microseconds, bool a_do_sleep)
{
    if (a_do_sleep)
    {
        usleep(a_microseconds);
    }
}
