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
    StepperMode m_stepperMode;
    int16_t m_target_position; // mm

private:
    uint16_t m_speed; // mm/s
    uint16_t m_max_accel; // mm/s²
    uint16_t m_max_current;// mA

public:
    StepperMotor(uint16_t speed, uint16_t accel, uint16_t max_current): m_speed(speed), m_max_accel(accel), m_max_current(max_current)
    {
        m_stepperMode = StepperMode::DISABLE;
        m_target_position = 0;
    };

    explicit  StepperMotor()
      : StepperMotor(100, 2000, 5000)
    {
    }
    //void publish(float a_speed, float a_angle);

    void setSpeed(uint16_t a_speed) {m_speed = a_speed;};
    void setEnable(StepperMode a_new_mode){m_stepperMode = a_new_mode;};
};

class StepperElevator: public StepperMotor
{
private:
  //float m_;
  bool m_homing_done;
  uint32_t m_max_height;
  uint32_t m_nb_tics_from_homing;
  //float m_mm_to_ticks; => in the µC

public:
    void doHoming(){m_stepperMode = StepperMode::HOMING;};
    bool homingDone(){return m_homing_done;};
    bool goToPosition(int32_t a_distance_in_mm);

    StepperElevator(uint32_t a_max_height_mm) : StepperMotor()
    {};

    StepperElevator(uint16_t speed, uint16_t accel, uint16_t max_current, uint32_t a_max_height_mm) : StepperMotor(speed, accel, max_current)
    {};
};