#pragma once
#include "rclcpp/rclcpp.hpp"

enum StepperMode
{
    DISABLE,
    POSITION,
    SPEED,
    HOMING
};

class StepperMotor
{
protected:
    StepperMode m_stepper_mode;
    int16_t m_target_position; // mm

private:
    uint16_t m_speed; // mm/s
    uint16_t m_max_accel; // mm/s²
    uint16_t m_max_current;// mA

public:
    StepperMotor(uint16_t speed, uint16_t accel, uint16_t max_current): m_speed(speed), m_max_accel(accel), m_max_current(max_current)
    {
        m_stepper_mode = StepperMode::DISABLE;
        m_target_position = 0;
    };

    explicit  StepperMotor()
      : StepperMotor(100, 2000, 5000)
    {
    }
    //void publish(float a_speed, float a_angle);

    void setSpeed(uint16_t a_speed) {m_speed = a_speed;};
    void setEnable(StepperMode a_new_mode){m_stepper_mode = a_new_mode;};

    uint16_t getAccel() {return m_max_accel;};
    uint16_t getSpeed() {return m_speed;};
    uint16_t getMaxCurrent() {return m_max_current;};
    uint16_t getTargetPosition() {return m_target_position;};
    StepperMode getStepperMode() {return m_stepper_mode;};
};

class StepperElevator: public StepperMotor
{
private:
  //float m_;
  bool m_homing_done;
  uint16_t m_max_height_mm;


public:
    void doHoming(){m_stepper_mode = StepperMode::HOMING;};
    bool homingDone(){return m_homing_done;};
    bool goToPosition(int16_t a_distance_in_mm);

    StepperElevator(uint16_t a_max_height_mm) : StepperMotor()
    {m_max_height_mm = a_max_height_mm;};

    StepperElevator(uint16_t speed, uint16_t accel, uint16_t max_current, uint16_t a_max_height_mm) : StepperMotor(speed, accel, max_current)
    {m_max_height_mm = a_max_height_mm;};
};