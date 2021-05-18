#pragma once
#include "krabilib/strategie/strategiev3.h"
#include "pump.h"
#include "servomotor.h"

enum GrabberContent
{
    NOTHING,
    GREEN_CUP,
    RED_CUP,
    UNKNOWN,
    ANY
};

class Grabber
{
public:
    Grabber(){};
    Grabber(Position a_relative_position,
            std::shared_ptr<Servomotor> a_servo,
            std::shared_ptr<Pump> a_pump);

    void release(GrabberContent type_to_release);
    void grab(GrabberContent type_to_catch);
    float getReach();
    Angle getAngle();

private:
    Position m_relative_position;
    GrabberContent m_content;
    std::shared_ptr<Servomotor> m_servo;
    std::shared_ptr<Pump> m_pump;

    void release();
    void grab();
};
