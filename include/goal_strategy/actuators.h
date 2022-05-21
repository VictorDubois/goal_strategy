#pragma once
#include "grabber.h"
#include "krabi_msgs/actuators.h"
#include "pump.h"
#include "servomotor.h"
#include <ros/ros.h>
#include <thread>

class Actuators

{
private:
    ros::Publisher m_pub;
    ros::NodeHandle* m_nh;
    krabi_msgs::actuators m_message;
    std::shared_ptr<Servomotor> m_servo_pusher;
    std::shared_ptr<Pump> m_fake_statuette_pump;
    std::shared_ptr<Servomotor> m_grabber_servo_base;
    std::shared_ptr<Servomotor> m_grabber_servo_mid;
    std::shared_ptr<Servomotor> m_grabber_servo_suction_cup;
    std::shared_ptr<Pump> m_grabber_pump;

    std::thread m_running;

    int m_score;
    bool m_shutdown;

    void run();

public:
    Actuators(){};
    Actuators(ros::NodeHandle* a_nh,
              std::string a_name,
              std::shared_ptr<Servomotor> a_servo_pusher,
              std::shared_ptr<Pump> a_fake_statuette_pump,
              std::shared_ptr<Servomotor> a_grabber_servo_base,
              std::shared_ptr<Servomotor> a_grabber_servo_mid,
              std::shared_ptr<Servomotor> a_grabber_servo_suction_cup,
              std::shared_ptr<Pump> a_grabber_pump);

    void publish();
    void start();
    void shutdown();
    void set_score(int a_score);
};
