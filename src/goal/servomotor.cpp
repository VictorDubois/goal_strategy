#include "../include/goal_strategy/servomotor.h"

Servomotor::Servomotor(float speed, float angle)
  : m_speed(speed)
  , m_angle(angle)
{
}

void Servomotor::setSpeed(float a_speed)
{
    m_speed = a_speed;
}
void Servomotor::setAngle(float a_angle)
{
    m_angle = a_angle;
}

float Servomotor::getSpeed()
{
    return m_speed;
}
float Servomotor::getAngle()
{
    return m_angle;
}
