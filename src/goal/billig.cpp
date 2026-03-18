#include <goal_strategy/billig.h>
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

Billig::Billig(Position a_relative_position_in_front,
               Position a_relative_position_inside,
               std::shared_ptr<Servomotor> a_servo_magnet_1,
               std::shared_ptr<Servomotor> a_servo_magnet_2,
               std::shared_ptr<Servomotor> a_servo_magnet_3,
               std::shared_ptr<Servomotor> a_servo_magnet_4,
               std::shared_ptr<AX12> a_ax12_1,
               std::shared_ptr<AX12> a_ax12_2,
               std::shared_ptr<AX12> a_ax12_3,
               std::shared_ptr<AX12> a_ax12_4,
               std::shared_ptr<StepperElevator> a_stepper_elevator,
               std::shared_ptr<Pump> a_pump_1,
               std::shared_ptr<Pump> a_pump_2,
               std::shared_ptr<Pump> a_pump_3,
               std::shared_ptr<Pump> a_pump_4)
  : m_relative_position_in_front(a_relative_position_in_front)
  , m_relative_position_inside(a_relative_position_inside)
  , m_relative_position(a_relative_position_in_front)
  , m_servo_magnet_1(a_servo_magnet_1)
  , m_servo_magnet_2(a_servo_magnet_2)
  , m_servo_magnet_3(a_servo_magnet_3)
  , m_servo_magnet_4(a_servo_magnet_4)
  , m_ax12_1(a_ax12_1)
  , m_ax12_2(a_ax12_2)
  , m_ax12_3(a_ax12_3)
  , m_ax12_4(a_ax12_4)
  , m_stepper_elevator(a_stepper_elevator)
  , m_pump_1(a_pump_1)
  , m_pump_2(a_pump_2)
  , m_pump_3(a_pump_3)
  , m_pump_4(a_pump_4)
{
}

void Billig::setInFront()
{
    m_relative_position = m_relative_position_in_front;
}
void Billig::setInside()
{
    m_relative_position = m_relative_position_inside;
}

/*
 * Returns the reach of Billig, with respect to the center of the robot
 */
float Billig::getReach()
{
    return m_relative_position.getNorme();
}

Angle Billig::getAngle()
{
    return m_relative_position.getAngle();
}

void Billig::resetElevatorLow()
{
    m_stepper_elevator->goToPosition(0); // mm
}

void Billig::initializeElevator()
{
    m_stepper_elevator->doHoming();
}

bool Billig::elevatorInitDone()
{
    return m_stepper_elevator->homingDone();
}

void Billig::auto_initBillig(bool a_first_elevator_init)
{
    auto init_billig_thread = std::thread(&Billig::initBillig, this, a_first_elevator_init);
    init_billig_thread.detach();
}

void Billig::initBillig(bool a_first_elevator_init)
{
    m_ax12_1->set(AX12_LEFT_GRAB, 100);
    m_ax12_2->set(AX12_RIGHT_GRAB, 100);
    m_ax12_3->set(AX12_SUCTION_HIGH, 100);
    m_ax12_4->set(AX12_SUCTION_HIGH, 100);
    m_servo_magnet_1->set(SERVO_RIGHTMOST_CAN_GRAB, 100);
    m_servo_magnet_2->set(SERVO_RIGHTCENTER_CAN_GRAB, 100);
    m_servo_magnet_3->set(SERVO_LEFTCENTER_CAN_GRAB, 100);
    m_servo_magnet_4->set(SERVO_LEFTMOST_CAN_GRAB, 100);
    m_pump_1->setPumping(false);
    m_pump_2->setPumping(false);
    m_pump_3->setPumping(false);
    m_pump_4->setPumping(false);

    if (!a_first_elevator_init)
    {
        m_stepper_elevator->goToPosition(0); // mm
        usleep(1.5e6);
    }
    initializeElevator();
}

bool Billig::grab_caisses(bool /*a_do_sleep*/)
{
    m_pump_1->setPumping(true);

    m_ax12_1->set(AX12_SUCTION_GRAB, 100);

    usleep(1.5e6);

    m_ax12_1->set(AX12_SUCTION_TRANSPORT, 100);

    // @todo check that the cans are grabbed

    return true;
}

bool Billig::allCaissesDetected()
{
    return m_caisse_detected == 0b1111;
}

bool Billig::elevatorInitHasFailed()
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

bool Billig::drop_caisses(bool /*a_do_sleep*/)
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

    m_ax12_1->set(AX12_SUCTION_TAKE_OUTER_CANS, 100);
    m_stepper_elevator->goToPosition(5); // mm

    usleep(0.5e6);

    m_ax12_1->set(AX12_LEFT_EXT, 10);
    m_ax12_2->set(AX12_RIGHT_EXT, 10);
    usleep(2.5e6);

    m_stepper_elevator->goToPosition(140); // mm

    usleep(2.0e6);
    m_ax12_1->set(AX12_SUCTION_HIGH, 100);
    usleep(0.5e6);

    m_ax12_1->set(AX12_LEFT_GRAB, 10);
    m_ax12_2->set(AX12_RIGHT_GRAB, 10);
    usleep(1.5e6);

    m_stepper_elevator->goToPosition(120); // mm

    usleep(1.0e6);
    m_pump_1->setPumping(false);
    m_pump_1->release();

    usleep(1.5e6);
    m_ax12_3->set(AX12_SUCTION_HIGH, 100);
    m_servo_magnet_1->set(SERVO_RIGHTMOST_CAN_RELEASE, 100);
    m_servo_magnet_2->set(SERVO_RIGHTCENTER_CAN_RELEASE, 100);
    m_servo_magnet_3->set(SERVO_LEFTCENTER_CAN_RELEASE, 100);
    m_servo_magnet_4->set(SERVO_LEFTMOST_CAN_RELEASE, 100);
    return success;
}

void Billig::conditionnal_sleep(uint a_microseconds, bool a_do_sleep)
{
    if (a_do_sleep)
    {
        usleep(a_microseconds);
    }
}
