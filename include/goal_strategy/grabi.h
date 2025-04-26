#pragma once
#include "AX12.h"
#include "krabilib/strategie/strategiev3.h"
#include "servomotor.h"
#include "stepper.h"

class Grabi
{
public:
    Grabi() {};
    Grabi(Position a_relative_position_in_front,
          Position a_relative_position_inside,
          std::shared_ptr<Servomotor> a_servo_magnet_1,
          std::shared_ptr<Servomotor> a_servo_magnet_2,
          std::shared_ptr<Servomotor> a_servo_magnet_3,
          std::shared_ptr<Servomotor> a_servo_magnet_4,
          std::shared_ptr<AX12> a_ax12_left_can,
          std::shared_ptr<AX12> a_ax12_right_can,
          std::shared_ptr<AX12> a_ax12_suction_cup,
          std::shared_ptr<AX12> a_ax12_lever,
          std::shared_ptr<StepperElevator> a_stepper_elevator);

    bool grab_plateforme(bool a_do_sleep = true);
    bool drop_plateforme(bool a_do_sleep = true);
    void setInFront();
    void setInside();
    float getReach();
    Angle getAngle();
    void initializeElevator();
    bool elevatorInitDone();

private:
    void conditionnal_sleep(uint a_microseconds, bool a_do_sleep);

    Position m_relative_position_in_front;
    Position m_relative_position_inside;
    Position m_relative_position;
    std::shared_ptr<Servomotor> m_servo_magnet_1;
    std::shared_ptr<Servomotor> m_servo_magnet_2;
    std::shared_ptr<Servomotor> m_servo_magnet_3;
    std::shared_ptr<Servomotor> m_servo_magnet_4;
    std::shared_ptr<AX12> m_ax12_left_can;
    std::shared_ptr<AX12> m_ax12_right_can;
    std::shared_ptr<AX12> m_ax12_suction_cup;
    std::shared_ptr<AX12> m_ax12_lever;
    std::shared_ptr<StepperElevator> m_stepper_elevator;
    bool elevatorInitHasFailed();
};
