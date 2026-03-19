#include "../include/goal_strategy/actuators2025.h"
#include <unistd.h> //usleep

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
                             std::shared_ptr<AX12> a_AX12_1,
                             std::shared_ptr<AX12> a_AX12_2,
                             std::shared_ptr<AX12> a_AX12_3,
                             std::shared_ptr<AX12> a_AX12_4,
                             std::shared_ptr<StepperElevator> a_stepper_elevator_1,
                             std::shared_ptr<Pump> a_pump_1,
                             std::shared_ptr<Pump> a_pump_2,
                             std::shared_ptr<Pump> a_pump_3,
                             std::shared_ptr<Pump> a_pump_4)
  : m_node(a_node)
  , m_servo_1(a_servo_1)
  , m_servo_2(a_servo_2)
  , m_servo_3(a_servo_3)
  , m_servo_4(a_servo_4)
  , m_servo_5(a_servo_5)
  , m_servo_6(a_servo_6)
  , m_servo_7(a_servo_7)
  , m_servo_8(a_servo_8)
  , m_AX12_1(a_AX12_1)
  , m_AX12_2(a_AX12_2)
  , m_AX12_3(a_AX12_3)
  , m_AX12_4(a_AX12_4)
  , m_stepper_elevator_1(a_stepper_elevator_1)
  , m_pump_1(a_pump_1)
  , m_pump_2(a_pump_2)
  , m_pump_3(a_pump_3)
  , m_pump_4(a_pump_4)
{
    m_pub = m_node->create_publisher<krabi_msgs::msg::Actuators2025>(a_name, 5);
    m_shutdown = false;
}

void Actuators2025::updateStepperElevator(krabi_msgs::msg::InfosStepper::SharedPtr stepper_info_msg)
{
    m_stepper_elevator_1->updateElevatorInfos(stepper_info_msg);
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

    m_message.ax12_1.mode = m_AX12_1->getMode();
    m_message.ax12_1.position = m_AX12_1->getAngle();
    m_message.ax12_1.max_speed = m_AX12_1->getSpeed();
    m_message.ax12_1.torque_enable = m_AX12_1->getTorqueEnable();
    m_message.ax12_1.temperature_limit = m_AX12_1->getTemperatureLimit();
    m_message.ax12_1.current_limit = m_AX12_1->getCurrentLimit();
    m_message.ax12_1.max_accel = m_AX12_1->getCurrentLimit();

    m_message.ax12_2.mode = m_AX12_2->getMode();
    m_message.ax12_2.position = m_AX12_2->getAngle();
    m_message.ax12_2.max_speed = m_AX12_2->getSpeed();
    m_message.ax12_2.torque_enable = m_AX12_2->getTorqueEnable();
    m_message.ax12_2.temperature_limit = m_AX12_2->getTemperatureLimit();
    m_message.ax12_2.current_limit = m_AX12_2->getCurrentLimit();
    m_message.ax12_2.max_accel = m_AX12_2->getCurrentLimit();

    m_message.ax12_3.mode = m_AX12_3->getMode();
    m_message.ax12_3.position = m_AX12_3->getAngle();
    m_message.ax12_3.max_speed = m_AX12_3->getSpeed();
    m_message.ax12_3.torque_enable = m_AX12_3->getTorqueEnable();
    m_message.ax12_3.temperature_limit = m_AX12_3->getTemperatureLimit();
    m_message.ax12_3.current_limit = m_AX12_3->getCurrentLimit();
    m_message.ax12_3.max_accel = m_AX12_3->getCurrentLimit();

    m_message.ax12_4.mode = m_AX12_4->getMode();
    m_message.ax12_4.position = m_AX12_4->getAngle();
    m_message.ax12_4.max_speed = m_AX12_4->getSpeed();
    m_message.ax12_4.torque_enable = m_AX12_4->getTorqueEnable();
    m_message.ax12_4.temperature_limit = m_AX12_4->getTemperatureLimit();
    m_message.ax12_4.current_limit = m_AX12_4->getCurrentLimit();
    m_message.ax12_4.max_accel = m_AX12_4->getCurrentLimit();

    m_message.vacuum_1.enable_pump = m_pump_1->getPumping();
    m_message.vacuum_1.release = m_pump_1->getRelease();
    m_message.vacuum_2.enable_pump = m_pump_2->getPumping();
    m_message.vacuum_2.release = m_pump_2->getRelease();
    m_message.vacuum_3.enable_pump = m_pump_3->getPumping();
    m_message.vacuum_3.release = m_pump_3->getRelease();
    m_message.vacuum_4.enable_pump = m_pump_4->getPumping();
    m_message.vacuum_4.release = m_pump_4->getRelease();

    m_message.stepper_1.speed = m_stepper_elevator_1->getSpeed();
    m_message.stepper_1.accel = m_stepper_elevator_1->getAccel();
    m_message.stepper_1.position = m_stepper_elevator_1->getTargetPosition();
    m_message.stepper_1.current = m_stepper_elevator_1->getMaxCurrent();
    m_message.stepper_1.mode = m_stepper_elevator_1->getStepperMode();

    m_message.stepper_2.mode = StepperMode::DISABLE;

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
        m_message.ax12_1.torque_enable = 0;
        m_message.ax12_2.torque_enable = 0;
        m_message.ax12_3.torque_enable = 0;
        m_message.ax12_4.torque_enable = 0;
        m_message.stepper_1.mode = StepperMode::DISABLE;
        m_message.stepper_2.mode = StepperMode::DISABLE;

        m_message.vacuum_1.enable_pump = false;
        m_message.vacuum_1.release = true;
        m_message.vacuum_2.enable_pump = false;
        m_message.vacuum_2.release = true;
        m_message.vacuum_3.enable_pump = false;
        m_message.vacuum_3.release = true;
        m_message.vacuum_4.enable_pump = false;
        m_message.vacuum_4.release = true;
    }

    m_pub->publish(m_message);
}
