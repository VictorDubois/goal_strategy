#pragma once
#include "krabilib/strategie/strategiev3.h"
#include "servomotor.h"

class Claws
{
public:
    Claws(){};
    Claws(Position a_relative_position,
          std::shared_ptr<Servomotor> a_left_servo,
          std::shared_ptr<Servomotor> a_right_servo);

    void grab_pile();
    void release_pile();
    void retract();
    float getReach();
    Angle getAngle();

private:
    Position m_relative_position;
    std::shared_ptr<Servomotor> m_left_servo;
    std::shared_ptr<Servomotor> m_right_servo;
};
