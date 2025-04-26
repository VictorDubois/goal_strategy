#pragma once
#include "AX12.h"
#include "claws.h"
#include "krabi_msgs/msg/actuators2025.hpp"
#include "pump.h"
#include "rclcpp/rclcpp.hpp"
#include "servomotor.h"
#include "stepper.h"
#include <thread>

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
    std::shared_ptr<Pump> m_grabber_pump;

    std::thread m_running;

    int m_score;
    bool m_shutdown;

    void run();

public:
    Actuators2025() {};
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
                  std::shared_ptr<Pump> a_grabber_pump);

    void publish();
    void start();
    void shutdown();
    void set_score(int a_score);
    void updateStepperElevator(krabi_msgs::msg::InfosStepper stepper_info_msg);
};
