#include "../include/goal_strategy/actuators.h"

Actuators::Actuators(rclcpp::Node a_node,
                     std::string a_name,
                     std::shared_ptr<Servomotor> a_servo_pusher,
                     std::shared_ptr<Pump> a_fake_statuette_pump,
                     std::shared_ptr<Servomotor> a_grabber_servo_base,
                     std::shared_ptr<Servomotor> a_grabber_servo_mid,
                     std::shared_ptr<Servomotor> a_grabber_servo_suction_cup,
                     std::shared_ptr<Pump> a_grabber_pump)
  : m_node(a_node)
  , m_servo_pusher(a_servo_pusher)
  , m_fake_statuette_pump(a_fake_statuette_pump)
  , m_grabber_servo_base(a_grabber_servo_base)
  , m_grabber_servo_mid(a_grabber_servo_mid)
  , m_grabber_servo_suction_cup(a_grabber_servo_suction_cup)
  , m_grabber_pump(a_grabber_pump)
{
    m_pub = node->create_publisher<krabi_msgs::msg::actuators>(a_name, 5);
  
    m_shutdown = false;
}

void Actuators::start()
{
    m_running = std::thread(&Actuators::run, this);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "actuators running");
}

void Actuators::run()
{
    while (true)
    {
        usleep(100000);
        publish();
    }
}

void Actuators::set_score(int a_score)
{
    m_score = a_score;
}

void Actuators::shutdown()
{
    m_shutdown = true;
}

void Actuators::publish()
{
    m_message.score = m_score;
    m_message.arm_base_servo.enable = true;
    m_message.arm_mid_servo.enable = true;
    m_message.arm_suction_cup_servo.enable = true;
    m_message.pusher_servo.enable = true;

    m_message.arm_base_servo.angle = m_grabber_servo_base->getAngle();
    m_message.arm_base_servo.speed = m_grabber_servo_base->getSpeed();

    m_message.arm_mid_servo.angle = m_grabber_servo_mid->getAngle();
    m_message.arm_mid_servo.speed = m_grabber_servo_mid->getSpeed();

    m_message.arm_suction_cup_servo.angle = m_grabber_servo_suction_cup->getAngle();
    m_message.arm_suction_cup_servo.speed = m_grabber_servo_suction_cup->getSpeed();

    m_message.arm_vacuum.enable_pump = m_grabber_pump->getPumping();
    m_message.arm_vacuum.release = m_grabber_pump->getRelease();

    m_message.pusher_servo.angle = m_servo_pusher->getAngle();
    m_message.pusher_servo.speed = m_servo_pusher->getSpeed();

    m_message.fake_statuette_vacuum.enable_pump = m_fake_statuette_pump->getPumping();
    m_message.fake_statuette_vacuum.release = m_fake_statuette_pump->getRelease();

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

    m_pub->publish(m_message);
}
