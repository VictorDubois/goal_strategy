#pragma once
#include "claws.h"
#include "krabi_msgs/actuators.h"
#include "pump.h"
#include "servomotor.h"
#include <ros/ros.h>
#include <thread>

class Actuators2023

{
private:
    ros::Publisher m_pub;
    ros::NodeHandle* m_nh;
    krabi_msgs::actuators m_message;
    std::shared_ptr<Servomotor> m_servo_cherries;
    std::shared_ptr<Servomotor> m_claw_servo_left;
    std::shared_ptr<Servomotor> m_claw_servo_right;

    std::thread m_running;

    int m_score;
    bool m_shutdown;
    bool m_disguise;

    void run();

public:
    Actuators2023(){};
    Actuators2023(ros::NodeHandle* a_nh,
                  std::string a_name,
                  std::shared_ptr<Servomotor> a_servo_cherries,
                  std::shared_ptr<Servomotor> a_claw_servo_left,
                  std::shared_ptr<Servomotor> a_claw_servo_right);

    void publish();
    void start();
    void shutdown();
    void set_score(int a_score);
    void disguise();
};
