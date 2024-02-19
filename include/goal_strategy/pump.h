#pragma once
#include "rclcpp/rclcpp.hpp"

class Pump
{
private:
    bool m_enable;
    bool m_release;

public:
    Pump(bool enable, bool release);
    Pump()
      : Pump(false, true)
    {
    }

    bool getPumping();
    void setPumping(bool a_enable);
    void release();
    bool getRelease();
};
