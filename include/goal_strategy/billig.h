#pragma once
#include "AX12.h"
#include "krabilib/strategie/strategiev3.h"
#include "pump.h"
#include "servomotor.h"
#include "stepper.h"

class Billig
{
public:
    Billig() {};
    Billig(Position a_relative_position_in_front,
           Position a_relative_position_inside,
           std::shared_ptr<Servomotor> a_servo_magnet_1,
           std::shared_ptr<Servomotor> a_servo_magnet_2,
           std::shared_ptr<Servomotor> a_servo_magnet_3,
           std::shared_ptr<Servomotor> a_servo_magnet_4,
           std::shared_ptr<AX12> a_ax12_1,
           std::shared_ptr<AX12> a_ax12_2,
           std::shared_ptr<AX12> a_ax12_3,
           std::shared_ptr<AX12> a_ax12_4,
           std::shared_ptr<StepperElevator> a_stepper_elevator,
           std::shared_ptr<Pump> a_pump_1,
           std::shared_ptr<Pump> a_pump_2,
           std::shared_ptr<Pump> a_pump_3,
           std::shared_ptr<Pump> a_pump_4);

    bool grab_caisses(bool a_do_sleep = true);
    bool drop_caisses(bool a_do_sleep = true);
    void auto_flip_caisses(bool leftmost_up,
                           bool leftcenter_up,
                           bool rightcenter_up,
                           bool rightmost_up);
    void reset_flipper();
    void auto_reset_flipper();
    bool flip_caisses(bool leftmost_up,
                      bool leftcenter_up,
                      bool rightcenter_up,
                      bool rightmost_up,
                      bool a_do_sleep = true);
    void setInFront();
    void setInside();
    float getReach();
    Angle getAngle();
    void initializeElevator();
    void auto_initBillig(bool a_force_init_elevator);

    void initBillig(bool a_force_init_elevator);

    bool elevatorInitDone();
    void updateCaisseDetected(uint8_t a_caisse_detected)
    {
        m_caisse_detected = a_caisse_detected;
    };
    bool allCaissesDetected();

    void updateAX12Infos(int16_t a_current_position,
                         uint8_t a_present_temperature,
                         uint16_t a_present_current,
                         uint8_t a_moving,
                         uint8_t AX12_ID)
    {
        switch (AX12_ID)
        {
        case 1:
            m_ax12_1->updateInfos(
              a_current_position, a_present_temperature, a_present_current, a_moving);
            break;
        case 2:
            m_ax12_2->updateInfos(
              a_current_position, a_present_temperature, a_present_current, a_moving);
            break;
        case 3:
            m_ax12_3->updateInfos(
              a_current_position, a_present_temperature, a_present_current, a_moving);
            break;
        case 4:
            m_ax12_4->updateInfos(
              a_current_position, a_present_temperature, a_present_current, a_moving);
            break;
        default:
            break;
        }
    };

private:
    void conditionnal_sleep(uint a_microseconds, bool a_do_sleep);
    void wait_for_mutex();

    Position m_relative_position_in_front;
    Position m_relative_position_inside;
    Position m_relative_position;
    std::shared_ptr<Servomotor> m_servo_magnet_1;
    std::shared_ptr<Servomotor> m_servo_magnet_2;
    std::shared_ptr<Servomotor> m_servo_magnet_3;
    std::shared_ptr<Servomotor> m_servo_magnet_4;
    std::shared_ptr<AX12> m_ax12_1;
    std::shared_ptr<AX12> m_ax12_2;
    std::shared_ptr<AX12> m_ax12_3;
    std::shared_ptr<AX12> m_ax12_4;
    std::shared_ptr<StepperElevator> m_stepper_elevator;
    std::shared_ptr<Pump> m_pump_1;
    std::shared_ptr<Pump> m_pump_2;
    std::shared_ptr<Pump> m_pump_3;
    std::shared_ptr<Pump> m_pump_4;

    bool elevatorInitHasFailed();
    uint8_t m_caisse_detected = 0;

    bool m_mutexTaken = false;
};
