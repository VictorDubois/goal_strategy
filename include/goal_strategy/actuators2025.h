#pragma once
#include "AX12.h"
#include "krabi_msgs/msg/actuators2025.hpp"
#include "pump.h"
#include "servomotor.h"
#include "stepper.h"
#include <thread>

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"

class Actuators2025

{
private:
    rclcpp::Publisher<krabi_msgs::msg::Actuators2025>::SharedPtr m_pub;

    rclcpp::Node::SharedPtr m_node;
    krabi_msgs::msg::Actuators2025 m_message;
    std::shared_ptr<Servomotor> m_servo_1;
    std::shared_ptr<Servomotor> m_servo_2;
    std::shared_ptr<Servomotor> m_servo_3;
    std::shared_ptr<Servomotor> m_servo_4;
    std::shared_ptr<Servomotor> m_servo_5;
    std::shared_ptr<Servomotor> m_servo_6;
    std::shared_ptr<Servomotor> m_servo_7;
    std::shared_ptr<Servomotor> m_servo_8;
    std::shared_ptr<AX12> m_AX12_1;
    std::shared_ptr<AX12> m_AX12_2;
    std::shared_ptr<AX12> m_AX12_3;
    std::shared_ptr<AX12> m_AX12_4;
    std::shared_ptr<StepperElevator> m_stepper_elevator_1;
    std::shared_ptr<Pump> m_pump_1;
    std::shared_ptr<Pump> m_pump_2;
    std::shared_ptr<Pump> m_pump_3;
    std::shared_ptr<Pump> m_pump_4;

    std::thread m_running;

    int m_score;
    bool m_shutdown = false;

    void run();

public:
    Actuators2025() {};
    ~Actuators2025();

    // A user-declared destructor suppresses the implicit move operations, but
    // GoalStrat's constructor move-assigns a temporary (m_actuators = Actuators2025(...)),
    // so re-enable them explicitly. Copy stays deleted via the std::thread member.
    Actuators2025(Actuators2025&&) = default;
    Actuators2025& operator=(Actuators2025&&) = default;

    Actuators2025(rclcpp::Node::SharedPtr a_node,
                  std::string a_name,
                  std::shared_ptr<Servomotor> m_servo_1,
                  std::shared_ptr<Servomotor> m_servo_2,
                  std::shared_ptr<Servomotor> m_servo_3,
                  std::shared_ptr<Servomotor> m_servo_4,
                  std::shared_ptr<Servomotor> m_servo_5,
                  std::shared_ptr<Servomotor> m_servo_6,
                  std::shared_ptr<Servomotor> m_servo_7,
                  std::shared_ptr<Servomotor> m_servo_8,
                  std::shared_ptr<AX12> m_AX12_1,
                  std::shared_ptr<AX12> m_AX12_2,
                  std::shared_ptr<AX12> m_AX12_3,
                  std::shared_ptr<AX12> m_AX12_4,
                  std::shared_ptr<StepperElevator> m_stepper_elevator_1,
                  std::shared_ptr<Pump> a_pump_1,
                  std::shared_ptr<Pump> a_pump_2,
                  std::shared_ptr<Pump> a_pump_3,
                  std::shared_ptr<Pump> a_pump_4);

    void publish();
    void start();
    void shutdown();
    void set_score(int a_score);
    void updateStepperElevator(krabi_msgs::msg::InfosStepper::SharedPtr stepper_info_msg);
};
