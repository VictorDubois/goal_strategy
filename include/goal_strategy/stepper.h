#pragma once
#include "rclcpp/rclcpp.hpp"

class StepperMotor
{
private:
    uint16_t m_speed; // mm/s
    uint16_t m_max_accel; // mm/s²
    bool m_enable;
    uint16_t m_max_current;// mA

public:
    StepperMotor(uint16_t speed, uint16_t accel, uint16_t max_current);
    explicit  StepperMotor()
      : StepperMotor(100, 2000, 5000)
    {
    }
    //void publish(float a_speed, float a_angle);

    void setSpeed(uint16_t a_speed) {m_speed = a_speed;};
    void setEnable(bool a_enable){m_enable = a_enable;};
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
    void doHoming();
    bool homingDone();
    void goToPosition(uint32_t a_distance_in_mm);

    StepperElevator(uint32_t a_max_height_mm) : StepperMotor()
    {};
};