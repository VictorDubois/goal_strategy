#pragma once
#include "AX12.h"
#include "krabilib/strategie/strategiev3.h"
#include "pump.h"
#include "servomotor.h"
#include "stepper.h"

#define AX12_LEFT_EXT 100
#define AX12_LEFT_INT 615
#define AX12_LEFT_GRAB 400

#define AX12_RIGHT_EXT 615
#define AX12_RIGHT_INT 100
#define AX12_RIGHT_GRAB 320

#define AX12_SUCTION_HIGH 828
#define AX12_SUCTION_LOW 340
#define AX12_SUCTION_GRAB 490
#define AX12_SUCTION_TRANSPORT 520       // todo: tune
#define AX12_SUCTION_TAKE_OUTER_CANS 600 // todo: tune

#define SERVO_RIGHTMOST_CAN_GRAB 67
#define SERVO_RIGHTMOST_CAN_RELEASE 144

#define SERVO_RIGHTCENTER_CAN_GRAB 52
#define SERVO_RIGHTCENTER_CAN_RELEASE 132

#define SERVO_LEFTCENTER_CAN_GRAB 67
#define SERVO_LEFTCENTER_CAN_RELEASE 144

#define SERVO_LEFTMOST_CAN_GRAB 62
#define SERVO_LEFTMOST_CAN_RELEASE 150

#define SERVO_FINGER_HIGH 100
#define SERVO_FINGER_HIGHER_TAKE_OUTER_CANS 100 // was 105 but bad idea
#define SERVO_FINGER_LOW 175
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
