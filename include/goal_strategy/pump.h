#pragma once
#include <ros/ros.h>

class Pump
{
private:
    bool m_enable;

public:
    Pump(bool enable);
    Pump()
      : Pump(false)
    {
    }

    bool getPumping();
    void setPumping(bool a_enable);
};
