#include <goal_strategy/grabi.h>

#define AX12_LEFT_EXT 100
#define AX12_LEFT_INT 615
#define AX12_LEFT_GRAB 400

#define AX12_RIGHT_EXT 615
#define AX12_RIGHT_INT 100
#define AX12_RIGHT_GRAB 320

#define AX12_SUCTION_HIGH 828
#define AX12_SUCTION_LOW 340
#define AX12_SUCTION_GRAB 490

#define SERVO_RIGHTMOST_CAN_GRAB 67
#define SERVO_RIGHTMOST_CAN_RELEASE 144

#define SERVO_RIGHTCENTER_CAN_GRAB 22
#define SERVO_RIGHTCENTER_CAN_RELEASE 132

#define SERVO_LEFTCENTER_CAN_GRAB 67
#define SERVO_LEFTCENTER_CAN_RELEASE 144

#define SERVO_LEFTMOST_CAN_GRAB 62
#define SERVO_LEFTMOST_CAN_RELEASE 150

#define SERVO_FINGER_HIGH 110
#define SERVO_FINGER_LOW 175

Grabi::Grabi(Position a_relative_position_in_front,
             Position a_relative_position_inside,
             std::shared_ptr<Servomotor> a_servo_magnet_1,
             std::shared_ptr<Servomotor> a_servo_magnet_2,
             std::shared_ptr<Servomotor> a_servo_magnet_3,
             std::shared_ptr<Servomotor> a_servo_magnet_4,
             std::shared_ptr<AX12> a_ax12_left_can,
             std::shared_ptr<AX12> a_ax12_right_can,
             std::shared_ptr<AX12> a_ax12_suction_cup,
             std::shared_ptr<Servomotor> a_servo_lever,
             std::shared_ptr<StepperElevator> a_stepper_elevator,
             std::shared_ptr<Pump> a_pump_plank)
  : m_relative_position_in_front(a_relative_position_in_front)
  , m_relative_position_inside(a_relative_position_inside)
  , m_relative_position(a_relative_position_in_front)
  , m_servo_magnet_1(a_servo_magnet_1)
  , m_servo_magnet_2(a_servo_magnet_2)
  , m_servo_magnet_3(a_servo_magnet_3)
  , m_servo_magnet_4(a_servo_magnet_4)
  , m_ax12_left_can(a_ax12_left_can)
  , m_ax12_right_can(a_ax12_right_can)
  , m_ax12_suction_cup(a_ax12_suction_cup)
  , m_servo_lever(a_servo_lever)
  , m_stepper_elevator(a_stepper_elevator)
  , m_pump_plank(a_pump_plank)
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

void Grabi::resetElevatorLow()
{
    m_stepper_elevator->goToPosition(0); // mm
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

    m_ax12_left_can->set(AX12_LEFT_GRAB, 100);
    m_ax12_right_can->set(AX12_RIGHT_GRAB, 100);
    m_ax12_suction_cup->set(AX12_SUCTION_HIGH, 100);

    m_servo_lever->set(SERVO_FINGER_LOW, 100);

    m_stepper_elevator->goToPosition(0); // mm
    usleep(1.5e6);
    if (m_stepper_elevator->movmentDone())
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Elevator ready on grab position");
    }
    else
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "Failed to get Elevator on grab position :(");
    }

    m_servo_magnet_1->set(SERVO_RIGHTMOST_CAN_GRAB, 100);
    m_servo_magnet_2->set(SERVO_RIGHTCENTER_CAN_GRAB, 100);
    m_servo_magnet_3->set(SERVO_LEFTCENTER_CAN_GRAB, 100);
    m_servo_magnet_4->set(SERVO_LEFTMOST_CAN_GRAB, 100);

    m_pump_plank->setPumping(true);

    usleep(1.5e6);

    m_ax12_suction_cup->set(AX12_SUCTION_LOW, 100);
    m_servo_lever->set(SERVO_FINGER_HIGH, 100);

    usleep(1.5e6);

    // @todo check that the cans are grabbed

    // m_stepper_elevator->goToPosition(150); // mm
    return true;
}

bool Grabi::allCansDetected()
{
    return m_can_detected == 0b1111;
}

bool Grabi::elevatorInitHasFailed()
{
    rclcpp::Time timeout = rclcpp::Clock{ RCL_STEADY_TIME }.now() + rclcpp::Duration(3, 0);

    while (rclcpp::Clock{ RCL_STEADY_TIME }.now() < timeout)
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
    bool success = true;

    if (elevatorInitHasFailed())
    {
        return false;
    }

    m_stepper_elevator->goToPosition(0); // mm
    /*rclcpp::Time timeout = rclcpp::Clock{ RCL_STEADY_TIME }.now() + rclcpp::Duration(3, 0);
    bool success = false;
    while (a_do_sleep && !success && rclcpp::Clock{ RCL_STEADY_TIME }.now() < timeout)
    {
        usleep(10);
        if (m_stepper_elevator->movmentDone())
        {
            success = true;
        }
    }*/
    m_ax12_left_can->set(AX12_LEFT_EXT, 100);
    m_ax12_left_can->set(AX12_RIGHT_EXT, 100);
    usleep(1.5e6);

    m_stepper_elevator->goToPosition(150); // mm

    usleep(2.5e6);

    m_ax12_left_can->set(AX12_LEFT_GRAB, 100);
    m_ax12_left_can->set(AX12_RIGHT_GRAB, 100);

    usleep(2.5e6);
    m_pump_plank->setPumping(false);
    m_pump_plank->release();
    m_servo_lever->set(SERVO_FINGER_LOW, 100);

    usleep(1.5e6);
    m_ax12_suction_cup->set(AX12_SUCTION_HIGH, 100);
    m_servo_magnet_1->set(SERVO_RIGHTMOST_CAN_RELEASE, 100);
    m_servo_magnet_2->set(SERVO_RIGHTCENTER_CAN_RELEASE, 100);
    m_servo_magnet_3->set(SERVO_LEFTCENTER_CAN_RELEASE, 100);
    m_servo_magnet_4->set(SERVO_LEFTMOST_CAN_RELEASE, 100);
    return success;
}

void Grabi::conditionnal_sleep(uint a_microseconds, bool a_do_sleep)
{
    if (a_do_sleep)
    {
        usleep(a_microseconds);
    }
}
