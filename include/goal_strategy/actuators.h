#pragma once
#include <ros/ros.h>
#include "krabi_msgs/actuators.h"
#include "servomotor.h"
#include "pump.h"
#include <thread>

class Actuators

{
private:
    ros::Publisher m_pub;
    ros::NodeHandle* m_nh;
    krabi_msgs::actuators m_message;
    std::shared_ptr<Servomotor> m_servo_balloon;
    std::shared_ptr<Pump> m_pump_balloon;
    std::shared_ptr<Servomotor> m_servo_vacuum_left;
    std::shared_ptr<Servomotor> m_servo_vacuum_middle;
    std::shared_ptr<Servomotor> m_servo_vacuum_right;
    std::thread m_running;

    void run();

public:
    Actuators(){};
    Actuators(ros::NodeHandle* m_nh, std::string a_name, std::shared_ptr<Servomotor> a_servo_balloon,
              std::shared_ptr<Pump> a_pump_balloon,
    std::shared_ptr<Servomotor> a_servo_vacuum_left,
    std::shared_ptr<Servomotor> a_servo_vacuum_middle,
    std::shared_ptr<Servomotor> a_servo_vacuum_right);

    void publish();
    void start();
};
