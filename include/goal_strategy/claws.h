#pragma once
#include "krabilib/strategie/strategiev3.h"
#include "servomotor.h"

class Claws
{
public:
    Claws(){};
    Claws(Position a_relative_position_in_front,
          Position a_relative_position_inside,
          std::shared_ptr<Servomotor> a_left_servo,
          std::shared_ptr<Servomotor> a_right_servo);

    void grab_pile(bool a_do_sleep = true);
    void release_pile(bool a_do_sleep = true);
    void retract(bool a_do_sleep = true);
    void setInFront();
    void setInside();
    float getReach();
    Angle getAngle();

private:
    void conditionnal_sleep(uint a_microseconds, bool a_do_sleep);

    Position m_relative_position_in_front;
    Position m_relative_position_inside;
    Position m_relative_position;
    std::shared_ptr<Servomotor> m_left_servo;
    std::shared_ptr<Servomotor> m_right_servo;
};
