#include "../include/goal_strategy/actuators2023.h"

Actuators2023::Actuators2023(rclcpp::Node::SharedPtr a_node,
                             std::string a_name,
                             std::shared_ptr<Servomotor> a_servo_cherries,
                             std::shared_ptr<Servomotor> a_claw_servo_left,
                             std::shared_ptr<Servomotor> a_claw_servo_right,
                             std::shared_ptr<Servomotor> a_grabber_servo_base,
                             std::shared_ptr<Servomotor> a_grabber_servo_mid,
                             std::shared_ptr<Servomotor> a_grabber_servo_suction_cup,
                             std::shared_ptr<Pump> a_grabber_pump)
  : m_node(a_node)
  , m_servo_cherries(a_servo_cherries)
  , m_claw_servo_left(a_claw_servo_left)
  , m_claw_servo_right(a_claw_servo_right)
  , m_grabber_servo_base(a_grabber_servo_base)
  , m_grabber_servo_mid(a_grabber_servo_mid)
  , m_grabber_servo_suction_cup(a_grabber_servo_suction_cup)
  , m_grabber_pump(a_grabber_pump)

{
    m_disguise = false;
    m_pub = m_node->create_publisher<krabi_msgs::msg::Actuators>(a_name, 5);
    m_shutdown = false;
}

void Actuators2023::start()
{
    m_running = std::thread(&Actuators2023::run, this);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "actuators 2023 running");
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

void Actuators2023::disguise()
{
    m_disguise = true;
}

void Actuators2023::publish()
{
    m_message.score = m_score;
    m_message.arm_base_servo.enable = true;
    m_message.arm_mid_servo.enable = true;
    m_message.arm_suction_cup_servo.enable = true;
    m_message.pusher_servo.enable = true;
    m_message.additionnal_servo_1.enable = true;
    m_message.additionnal_servo_2.enable = true;

    m_message.arm_base_servo.angle = m_grabber_servo_base->getAngle();
    m_message.arm_base_servo.speed = m_grabber_servo_base->getSpeed();

    m_message.arm_mid_servo.angle = m_grabber_servo_mid->getAngle();
    m_message.arm_mid_servo.speed = m_grabber_servo_mid->getSpeed();

    m_message.arm_suction_cup_servo.angle = m_grabber_servo_suction_cup->getAngle();
    m_message.arm_suction_cup_servo.speed = m_grabber_servo_suction_cup->getSpeed();

    m_message.arm_vacuum.enable_pump = m_grabber_pump->getPumping();
    m_message.arm_vacuum.release = m_grabber_pump->getRelease();

    m_message.pusher_servo.angle = m_servo_cherries->getAngle();
    m_message.pusher_servo.speed = m_servo_cherries->getSpeed();

    m_message.fake_statuette_vacuum.enable_pump = m_disguise;
    m_message.fake_statuette_vacuum.release = false;

    m_message.additionnal_servo_1.angle = m_claw_servo_right->getAngle();
    m_message.additionnal_servo_1.speed = m_claw_servo_right->getSpeed(); 
    m_message.additionnal_servo_2.angle = m_claw_servo_left->getAngle();
    m_message.additionnal_servo_2.speed = m_claw_servo_left->getSpeed();

    if (m_shutdown)
    {
        m_message.arm_base_servo.enable = false;
        m_message.arm_mid_servo.enable = false;
        m_message.arm_suction_cup_servo.enable = false;
        m_message.pusher_servo.enable = false;
        m_message.arm_vacuum.enable_pump = false;
        m_message.arm_vacuum.release = true;
        m_message.fake_statuette_vacuum.enable_pump = m_disguise;
        m_message.fake_statuette_vacuum.release = true;
    }

    m_pub->publish(m_message);
}
