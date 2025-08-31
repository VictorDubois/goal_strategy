#pragma once
#include "AX12.h"
#include "krabilib/strategie/strategiev3.h"
#include "pump.h"
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
          std::shared_ptr<Servomotor> a_servo_lever,
          std::shared_ptr<StepperElevator> a_stepper_elevator,
          std::shared_ptr<Pump> a_pump_plank);

    bool grab_plateforme(bool a_do_sleep = true);
    bool drop_plateforme(bool a_do_sleep = true);
    void setInFront();
    void setInside();
    float getReach();
    Angle getAngle();
    void initializeElevator();
    void resetElevatorLow();
    void auto_initGrabi(bool a_force_init_elevator);

    void initGrabi(bool a_force_init_elevator);

    bool elevatorInitDone();
    void updateCanDetected(uint8_t a_can_detected)
    {
        m_can_detected = a_can_detected;
    };
    bool allCansDetected();

    void updateAX12Infos(int16_t a_current_position,
                         uint8_t a_present_temperature,
                         uint16_t a_present_current,
                         uint8_t a_moving,
                         uint8_t AX12_ID)
    {
        switch (AX12_ID)
        {
        case 1:
            m_ax12_left_can->updateInfos(
              a_current_position, a_present_temperature, a_present_current, a_moving);
            break;
        case 2:
            m_ax12_right_can->updateInfos(
              a_current_position, a_present_temperature, a_present_current, a_moving);
            break;
        case 3:
            m_ax12_suction_cup->updateInfos(
              a_current_position, a_present_temperature, a_present_current, a_moving);
            break;
        case 4:
            break;
        default:
            break;
        }
    };

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
    std::shared_ptr<Servomotor> m_servo_lever;
    std::shared_ptr<StepperElevator> m_stepper_elevator;
    std::shared_ptr<Pump> m_pump_plank;

    bool elevatorInitHasFailed();
    uint8_t m_can_detected = 0;
};
