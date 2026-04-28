#include <goal_strategy/billig.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <thread>

#define AX12_LEFTMOST_GRAB 100
#define AX12_LEFTMOST_RELEASE 100

#define AX12_LEFTCENTER_GRAB 100
#define AX12_LEFTCENTER_RELEASE 100

#define AX12_RIGHTCENTER_GRAB 100
#define AX12_RIGHTCENTER_RELEASE 100

#define AX12_RIGHTMOST_GRAB 100
#define AX12_RIGHTMOST_RELEASE 100

#define SERVO_RIGHTMOST_UP 0
#define SERVO_RIGHTMOST_DOWN 180

#define SERVO_RIGHTCENTER_UP 0
#define SERVO_RIGHTCENTER_DOWN 180

#define SERVO_LEFTCENTER_UP 0
#define SERVO_LEFTCENTER_DOWN 180

#define SERVO_LEFTMOST_UP 0
#define SERVO_LEFTMOST_DOWN 180

enum ElevatorPositionMM // Positions in millimiters, with respect to the top of the elevator
                        //(0 when the elevator is at its highest position)
{
    GRABERS_LEVEL = -100,
    ABOVE_GRABBERS = 0,
    TRANSPORT = -50,
    CATCH = -150,
    RELEASE = -145
};

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

void Billig::auto_flip_caisses(bool leftmost_up,
                               bool leftcenter_up,
                               bool rightcenter_up,
                               bool rightmost_up)
{
    auto flip_caisses_thread = std::thread(
      &Billig::flip_caisses, this, leftmost_up, leftcenter_up, rightcenter_up, rightmost_up, true);
    flip_caisses_thread.detach();
}

void Billig::initBillig(bool a_first_elevator_init)
{
    if (a_first_elevator_init)
    {
        initializeElevator();
    }

    m_stepper_elevator->goToPosition(ElevatorPositionMM::ABOVE_GRABBERS);
    usleep(1.5e6);

    m_pump_1->setPumping(false);
    m_pump_2->setPumping(false);
    m_pump_3->setPumping(false);
    m_pump_4->setPumping(false);
    usleep(1.5e6);

    // close grabbers before turning
    m_ax12_1->set(AX12_LEFTMOST_GRAB, 100);
    m_ax12_2->set(AX12_LEFTCENTER_GRAB, 100);
    m_ax12_3->set(AX12_RIGHTCENTER_GRAB, 100);
    m_ax12_4->set(AX12_RIGHTMOST_GRAB, 100);

    usleep(1.5e6);

    m_servo_magnet_1->set(SERVO_RIGHTMOST_UP, 100);
    m_servo_magnet_2->set(SERVO_RIGHTCENTER_UP, 100);
    m_servo_magnet_3->set(SERVO_LEFTCENTER_UP, 100);
    m_servo_magnet_4->set(SERVO_LEFTMOST_UP, 100);
    usleep(1.5e6);

    m_ax12_1->set(AX12_LEFTMOST_RELEASE, 100);
    m_ax12_2->set(AX12_LEFTCENTER_RELEASE, 100);
    m_ax12_3->set(AX12_RIGHTCENTER_RELEASE, 100);
    m_ax12_4->set(AX12_RIGHTMOST_RELEASE, 100);

    m_stepper_elevator->goToPosition(ElevatorPositionMM::TRANSPORT);
    usleep(1.5e6);
}

bool Billig::grab_caisses(bool /*a_do_sleep*/)
{
    m_ax12_1->set(AX12_LEFTMOST_RELEASE, 100);
    m_ax12_2->set(AX12_LEFTCENTER_RELEASE, 100);
    m_ax12_3->set(AX12_RIGHTCENTER_RELEASE, 100);
    m_ax12_4->set(AX12_RIGHTMOST_RELEASE, 100);

    m_pump_1->setPumping(true);
    m_pump_2->setPumping(true);
    m_pump_3->setPumping(true);
    m_pump_4->setPumping(true);

    usleep(0.5e6);

    m_stepper_elevator->goToPosition(ElevatorPositionMM::CATCH); // mm

    usleep(1.5e6);

    m_stepper_elevator->goToPosition(ElevatorPositionMM::GRABERS_LEVEL); // mm

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

    m_stepper_elevator->goToPosition(ElevatorPositionMM::RELEASE);

    usleep(0.5e6);

    m_pump_1->setPumping(false);
    m_pump_2->setPumping(false);
    m_pump_3->setPumping(false);
    m_pump_4->setPumping(false);
    usleep(1.5e6);
    m_stepper_elevator->goToPosition(ElevatorPositionMM::TRANSPORT);

    return success;
}

bool Billig::reset_flipper(bool /*a_do_sleep*/)
{
    bool success = true;
    m_stepper_elevator->goToPosition(ElevatorPositionMM::ABOVE_GRABBERS);
    m_pump_1->release();
    m_pump_2->release();
    m_pump_3->release();
    m_pump_4->release();
    usleep(1.5e6);

    m_ax12_1->set(AX12_LEFTMOST_GRAB, 100);
    m_ax12_2->set(AX12_LEFTCENTER_GRAB, 100);
    m_ax12_3->set(AX12_RIGHTCENTER_GRAB, 100);
    m_ax12_4->set(AX12_RIGHTMOST_GRAB, 100);

    usleep(1.5e6);

    m_servo_magnet_1->set(SERVO_LEFTMOST_UP, 100);
    m_servo_magnet_2->set(SERVO_LEFTCENTER_UP, 100);
    m_servo_magnet_3->set(SERVO_RIGHTCENTER_UP, 100);
    m_servo_magnet_4->set(SERVO_RIGHTMOST_UP, 100);

    usleep(1.5e6);

    m_ax12_1->set(AX12_LEFTMOST_RELEASE, 100);
    m_ax12_2->set(AX12_LEFTCENTER_RELEASE, 100);
    m_ax12_3->set(AX12_RIGHTCENTER_RELEASE, 100);
    m_ax12_4->set(AX12_RIGHTMOST_RELEASE, 100);

    return success;
}

bool Billig::flip_caisses(bool leftmost_up,
                          bool leftcenter_up,
                          bool rightcenter_up,
                          bool rightmost_up,
                          bool /*a_do_sleep*/)
{
    bool success = true;
    m_stepper_elevator->goToPosition(ElevatorPositionMM::GRABERS_LEVEL);
    usleep(1.5e6);

    m_ax12_1->set(AX12_LEFTMOST_GRAB, 100);
    m_ax12_2->set(AX12_LEFTCENTER_GRAB, 100);
    m_ax12_3->set(AX12_RIGHTCENTER_GRAB, 100);
    m_ax12_4->set(AX12_RIGHTMOST_GRAB, 100);
    usleep(1.5e6);

    m_pump_1->setPumping(false);
    m_pump_2->setPumping(false);
    m_pump_3->setPumping(false);
    m_pump_4->setPumping(false);
    usleep(1.5e6);
    m_stepper_elevator->goToPosition(ElevatorPositionMM::ABOVE_GRABBERS);
    usleep(1.5e6);

    if (leftmost_up)
    {
        m_servo_magnet_1->set(SERVO_LEFTMOST_UP, 100);
    }
    else
    {
        m_servo_magnet_1->set(SERVO_LEFTMOST_DOWN, 100);
    }

    if (leftcenter_up)
    {
        m_servo_magnet_2->set(SERVO_LEFTCENTER_UP, 100);
    }
    else
    {
        m_servo_magnet_2->set(SERVO_LEFTCENTER_DOWN, 100);
    }

    if (rightcenter_up)
    {
        m_servo_magnet_3->set(SERVO_RIGHTCENTER_UP, 100);
    }
    else
    {
        m_servo_magnet_3->set(SERVO_RIGHTCENTER_DOWN, 100);
    }

    if (rightmost_up)
    {
        m_servo_magnet_4->set(SERVO_RIGHTMOST_UP, 100);
    }
    else
    {
        m_servo_magnet_4->set(SERVO_RIGHTMOST_DOWN, 100);
    }

    usleep(1.5e6);

    m_pump_1->setPumping(true);
    m_pump_2->setPumping(true);
    m_pump_3->setPumping(true);
    m_pump_4->setPumping(true);
    usleep(0.5e6);
    m_stepper_elevator->goToPosition(ElevatorPositionMM::GRABERS_LEVEL);

    usleep(1.5e6);

    m_ax12_1->set(AX12_LEFTMOST_RELEASE, 100);
    m_ax12_2->set(AX12_LEFTCENTER_RELEASE, 100);
    m_ax12_3->set(AX12_RIGHTCENTER_RELEASE, 100);
    m_ax12_4->set(AX12_RIGHTMOST_RELEASE, 100);

    return success;
}

void Billig::conditionnal_sleep(uint a_microseconds, bool a_do_sleep)
{
    if (a_do_sleep)
    {
        usleep(a_microseconds);
    }
}
