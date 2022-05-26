#pragma once
#include "krabilib/strategie/strategiev3.h"
#include "pump.h"
#include "servomotor.h"

enum GrabberContent
{
    NOTHING,
    GREEN_HEXA,
    RED_HEXA,
    BLUE_HEXA,
    UNKNOWN,
    ANY
};

class Grabber
{
public:
    Grabber(){};
    Grabber(Position a_relative_position,
            std::shared_ptr<Servomotor> a_servo_base,
            std::shared_ptr<Servomotor> a_servo_mid,
            std::shared_ptr<Servomotor> a_servo_suction_cup,
            std::shared_ptr<Pump> a_pump);

    void release_hexagon_on_ground(GrabberContent type_to_release);
    void release_hexagon_on_ground();
    void grab_hexagon(GrabberContent type_to_catch);
    void grab_statuette();
    void release_statuette();
    float getReach();
    Angle getAngle();

private:
    Position m_relative_position;
    GrabberContent m_content;
    std::shared_ptr<Servomotor> m_servo_base;
    std::shared_ptr<Servomotor> m_servo_mid;
    std::shared_ptr<Servomotor> m_servo_suction_cup;
    std::shared_ptr<Pump> m_pump;

    void grab_hexagon();
    void retract_arm();
};
