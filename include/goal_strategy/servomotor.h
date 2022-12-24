#pragma once
#include <ros/ros.h>

class Servomotor
{
private:
    float m_speed;
    float m_angle;

public:
    Servomotor(float speed, float angle);
    Servomotor()
      : Servomotor(0, 128)
    {
    }
    void publish(float a_speed, float a_angle);
    float getSpeed();
    float getAngle();
    void setAngle(float a_angle);
    void setSpeed(float a_speed);
    void set(float a_angle, float a_speed);
};
