#include <goal_strategy/grabi.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <thread>

#define AX12_LEFT_EXT 100
#define AX12_LEFT_INT 615
#define AX12_LEFT_GRAB 400

#define AX12_RIGHT_EXT 615
#define AX12_RIGHT_INT 101
#define AX12_RIGHT_GRAB 321

#define AX12_SUCTION_HIGH 828
#define AX12_SUCTION_LOW 340
#define AX12_SUCTION_GRAB 490
#define AX12_SUCTION_TRANSPORT 520       // todo: tune
#define AX12_SUCTION_TAKE_OUTER_CANS 600 // todo: tune

#define SERVO_RIGHTMOST_CAN_GRAB 67
#define SERVO_RIGHTMOST_CAN_RELEASE 144

#define SERVO_RIGHTCENTER_CAN_GRAB 52
#define SERVO_RIGHTCENTER_CAN_RELEASE 132

#define SERVO_LEFTCENTER_CAN_GRAB 67
#define SERVO_LEFTCENTER_CAN_RELEASE 144

#define SERVO_LEFTMOST_CAN_GRAB 62
#define SERVO_LEFTMOST_CAN_RELEASE 150

#define SERVO_FINGER_HIGH 100
#define SERVO_FINGER_HIGHER_TAKE_OUTER_CANS 100 // was 105 but bad idea
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

void Grabi::auto_initGrabi(bool a_first_elevator_init)
{
    auto init_grabi_thread = std::thread(&Grabi::initGrabi, this, a_first_elevator_init);
    init_grabi_thread.detach();
}

void Grabi::initGrabi(bool a_first_elevator_init)
{
    m_ax12_left_can->set(AX12_LEFT_GRAB, 100);
    m_ax12_right_can->set(AX12_RIGHT_GRAB, 100);
    m_ax12_suction_cup->set(AX12_SUCTION_HIGH, 100);
    m_servo_magnet_1->set(SERVO_RIGHTMOST_CAN_GRAB, 100);
    m_servo_magnet_2->set(SERVO_RIGHTCENTER_CAN_GRAB, 100);
    m_servo_magnet_3->set(SERVO_LEFTCENTER_CAN_GRAB, 100);
    m_servo_magnet_4->set(SERVO_LEFTMOST_CAN_GRAB, 100);
    m_servo_lever->set(SERVO_FINGER_LOW, 100);
    m_pump_plank->setPumping(false);

    if (!a_first_elevator_init)
    {
        m_stepper_elevator->goToPosition(0); // mm
        usleep(1.5e6);
    }
    initializeElevator();
}

bool Grabi::grab_plateforme(bool /*a_do_sleep*/)
{
    m_pump_plank->setPumping(true);

    m_ax12_suction_cup->set(AX12_SUCTION_GRAB, 100);
    m_servo_lever->set(SERVO_FINGER_HIGH, 100);

    usleep(1.5e6);

    m_ax12_suction_cup->set(AX12_SUCTION_TRANSPORT, 100);

    // @todo check that the cans are grabbed

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

bool Grabi::drop_plateforme(bool /*a_do_sleep*/)
{
    bool success = true;

    /*if (elevatorInitHasFailed())
    {
        return false;
    }*/

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

    m_ax12_left_can->set(AX12_SUCTION_TAKE_OUTER_CANS, 100);
    m_stepper_elevator->goToPosition(5); // mm

    usleep(0.5e6);
    m_servo_lever->set(SERVO_FINGER_HIGHER_TAKE_OUTER_CANS, 100);
    usleep(0.5e6);

    m_ax12_left_can->set(AX12_LEFT_EXT, 10);
    m_ax12_right_can->set(AX12_RIGHT_EXT, 10);
    usleep(2.5e6);

    m_stepper_elevator->goToPosition(140); // mm
    m_servo_lever->set(SERVO_FINGER_LOW, 100);

    usleep(2.0e6);
    m_ax12_left_can->set(AX12_SUCTION_HIGH, 100);
    usleep(0.5e6);

    m_ax12_left_can->set(AX12_LEFT_GRAB, 10);
    m_ax12_right_can->set(AX12_RIGHT_GRAB, 10);
    usleep(1.5e6);

    m_stepper_elevator->goToPosition(120); // mm

    usleep(1.0e6);
    m_pump_plank->setPumping(false);
    m_pump_plank->release();

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
