#include "../include/goal_strategy/actuators2023.h"

Actuators2023::Actuators2023(ros::NodeHandle* a_nh,
                             std::string a_name,
                             std::shared_ptr<Servomotor> a_servo_cherries,
                             std::shared_ptr<Servomotor> a_claw_servo_left,
                             std::shared_ptr<Servomotor> a_claw_servo_right)
  : m_nh(a_nh)
  , m_servo_cherries(a_servo_cherries)
  , m_claw_servo_left(a_claw_servo_left)
  , m_claw_servo_right(a_claw_servo_right)

{
    m_pub = m_nh->advertise<krabi_msgs::actuators>(a_name, 5);
    m_shutdown = false;
}

void Actuators2023::start()
{
    m_running = std::thread(&Actuators2023::run, this);
    ROS_INFO("actuators 2023 running");
}

void Actuators2023::run()
{
    while (true)
    {
        usleep(100000);
        publish();
    }
}

void Actuators2023::set_score(int a_score)
{
    m_score = a_score;
}

void Actuators2023::shutdown()
{
    m_shutdown = true;
}

void Actuators2023::publish()
{
    m_message.score = m_score;
    m_message.arm_base_servo.enable = true;
    m_message.arm_mid_servo.enable = true;
    m_message.arm_suction_cup_servo.enable = true;
    m_message.pusher_servo.enable = true;

    m_message.arm_base_servo.angle = m_claw_servo_right->getAngle();
    m_message.arm_base_servo.speed = m_claw_servo_right->getSpeed();

    m_message.arm_mid_servo.angle = m_claw_servo_left->getAngle();
    m_message.arm_mid_servo.speed = m_claw_servo_left->getSpeed();

    m_message.arm_suction_cup_servo.angle = 90;
    m_message.arm_suction_cup_servo.speed = 0;

    m_message.arm_vacuum.enable_pump = false;
    m_message.arm_vacuum.release = false;

    m_message.pusher_servo.angle = m_servo_cherries->getAngle();
    m_message.pusher_servo.speed = m_servo_cherries->getSpeed();

    m_message.fake_statuette_vacuum.enable_pump = false;
    m_message.fake_statuette_vacuum.release = false;

    if (m_shutdown)
    {
        m_message.arm_base_servo.enable = false;
        m_message.arm_mid_servo.enable = false;
        m_message.arm_suction_cup_servo.enable = false;
        m_message.pusher_servo.enable = false;
        m_message.arm_vacuum.enable_pump = false;
        m_message.arm_vacuum.release = true;
        m_message.fake_statuette_vacuum.enable_pump = false;
        m_message.fake_statuette_vacuum.release = true;
    }

    m_pub.publish(m_message);
}
