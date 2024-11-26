#include "../include/goal_strategy/actuators2025.h"

Actuators2025::Actuators2025(rclcpp::Node::SharedPtr a_node,
                             std::string a_name,
                             std::shared_ptr<Servomotor> a_servo_1,
                             std::shared_ptr<Servomotor> a_servo_2,
                             std::shared_ptr<Servomotor> a_servo_3,
                             std::shared_ptr<Servomotor> a_servo_4,
                             std::shared_ptr<Servomotor> a_servo_5,
                             std::shared_ptr<Servomotor> a_servo_6,
                             std::shared_ptr<Servomotor> a_servo_7,
                             std::shared_ptr<Servomotor> a_servo_8,
                             std::shared_ptr<StepperElevator> a_stepper_elevator_1,
                             std::shared_ptr<Pump> a_grabber_pump)
  : m_node(a_node)
  , m_servo_1(a_servo_1)
    , m_servo_2(a_servo_2)
    , m_servo_3(a_servo_3)
    , m_servo_4(a_servo_4)
    , m_servo_5(a_servo_5)
    , m_servo_6(a_servo_6)
    , m_servo_7(a_servo_7)
    , m_servo_8(a_servo_8)
    , m_stepper_elevator_1(a_stepper_elevator_1)
    , m_grabber_pump(a_grabber_pump)
{
    m_pub = m_node->create_publisher<krabi_msgs::msg::Actuators2025>(a_name, 5);
    m_shutdown = false;
}

void Actuators2025::start()
{
    m_running = std::thread(&Actuators2025::run, this);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "actuators 2025 running");
}

void Actuators2025::run()
{
    while (true)
    {
        usleep(100000);
        publish();
    }
}

void Actuators2025::set_score(int a_score)
{
    m_score = a_score;
}

void Actuators2025::shutdown()
{
    m_shutdown = true;
}

void Actuators2025::disguise()
{
}

void Actuators2025::publish()
{
    
    m_message.score = m_score;
    m_message.servo_1.enable = true;
    m_message.servo_2.enable = true;
    m_message.servo_3.enable = true;
    m_message.servo_4.enable = true;
    m_message.servo_5.enable = true;
    m_message.servo_6.enable = true;
    m_message.servo_7.enable = true;
    m_message.servo_8.enable = true;

    m_message.servo_1.angle = m_servo_1->getAngle();
    m_message.servo_1.speed = m_servo_1->getSpeed();
    m_message.servo_2.angle = m_servo_2->getAngle();
    m_message.servo_2.speed = m_servo_2->getSpeed();
    m_message.servo_3.angle = m_servo_3->getAngle();
    m_message.servo_3.speed = m_servo_3->getSpeed();
    m_message.servo_4.angle = m_servo_4->getAngle();
    m_message.servo_4.speed = m_servo_4->getSpeed();
    m_message.servo_5.angle = m_servo_5->getAngle();
    m_message.servo_5.speed = m_servo_5->getSpeed();
    m_message.servo_6.angle = m_servo_6->getAngle();
    m_message.servo_6.speed = m_servo_6->getSpeed();
    m_message.servo_7.angle = m_servo_7->getAngle();
    m_message.servo_7.speed = m_servo_7->getSpeed();
    m_message.servo_8.angle = m_servo_8->getAngle();
    m_message.servo_8.speed = m_servo_8->getSpeed();

    m_message.vacuum_1.enable_pump = false;
    m_message.vacuum_1.release = false;
    m_message.vacuum_2.enable_pump = false;
    m_message.vacuum_2.release = false;

    m_message.stepper_elevator_1.speed = m_stepper_elevator_1->getSpeed();
    m_message.stepper_elevator_1.accel = m_stepper_elevator_1->getAccel();
    m_message.stepper_elevator_1.position = m_stepper_elevator_1;
    m_message.stepper_elevator_1.current = m_stepper_elevator_1;
    m_message.stepper_elevator_1.mode = m_stepper_elevator_1;

    m_message.stepper_elevator_2.enable = true;

    if (m_shutdown)
    {
        m_message.servo_1.enable = false;
        m_message.servo_2.enable = false;
        m_message.servo_3.enable = false;
        m_message.servo_4.enable = false;
        m_message.servo_5.enable = false;
        m_message.servo_6.enable = false;
        m_message.servo_7.enable = false;
        m_message.servo_8.enable = false;
    }

    m_pub->publish(m_message);
}
