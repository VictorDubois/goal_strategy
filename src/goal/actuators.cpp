#include "../include/goal_strategy/actuators.h"

Actuators::Actuators(ros::NodeHandle* a_nh,
                     std::string a_name,
                     std::shared_ptr<Servomotor> a_servo_balloon,
                     std::shared_ptr<Pump> a_pump_balloon,
                     std::shared_ptr<Servomotor> a_servo_vacuum_left,
                     std::shared_ptr<Servomotor> a_servo_vacuum_middle,
                     std::shared_ptr<Servomotor> a_servo_vacuum_right)
  : m_nh(a_nh)
  , m_servo_balloon(a_servo_balloon)
  , m_pump_balloon(a_pump_balloon)
  , m_servo_vacuum_left(a_servo_vacuum_left)
  , m_servo_vacuum_middle(a_servo_vacuum_middle)
  , m_servo_vacuum_right(a_servo_vacuum_right)
{
    m_pub = m_nh->advertise<krabi_msgs::actuators>(a_name, 5);

    m_message.balloon_pump.enable_pump = false;
    m_message.balloon_pump.inflate_green = true;
    m_message.balloon_pump.inflate_red = true;
    m_message.balloon_servo.angle = 128;
    m_message.balloon_servo.speed = 0;
    m_message.pavillons.angle = 128;
    m_message.pavillons.speed = 0;
    m_message.phare_arm.angle = 128;
    m_message.phare_arm.speed = 0;
    m_message.vacuum_pump_left.enable_pump = false;
    m_message.vacuum_pump_middle.enable_pump = false;
    m_message.vacuum_pump_right.enable_pump = false;
}

void Actuators::start()
{
    m_running = std::thread(&Actuators::run, this);
    ROS_INFO("actuators running");
}

void Actuators::run()
{
    while (true)
    {
        usleep(100000);
        publish();
    }
}

void Actuators::publish()
{
    m_message.balloon_servo.angle = m_servo_balloon->getAngle();
    m_message.balloon_servo.speed = m_servo_balloon->getSpeed();
    m_message.balloon_pump.enable_pump = m_pump_balloon->getPumping();

    m_message.vacuum_servo_left.angle = m_servo_vacuum_left->getAngle();
    m_message.vacuum_servo_left.speed = m_servo_vacuum_left->getSpeed();

    m_message.vacuum_servo_middle.angle = m_servo_vacuum_middle->getAngle();
    m_message.vacuum_servo_middle.speed = m_servo_vacuum_middle->getSpeed();

    m_message.vacuum_servo_right.angle = m_servo_vacuum_right->getAngle();
    m_message.vacuum_servo_right.speed = m_servo_vacuum_right->getSpeed();

    m_pub.publish(m_message);
}
