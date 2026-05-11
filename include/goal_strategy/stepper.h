#pragma once
#include "krabi_msgs/msg/infos_stepper.hpp"
#include <chrono>
#include <thread>
#include <time.h>
#include <unistd.h>

enum StepperMode
{
    DISABLE = 0,
    POSITION = 10,
    SPEED = 20,
    HOMING = 30
};

class StepperMotor
{
protected:
    StepperMode m_stepper_mode;
    int16_t m_target_position; // mm
    int16_t m_distance_to_go;  // mm
    uint16_t m_homing_sequences_done;
    void updateInfos(krabi_msgs::msg::InfosStepper::SharedPtr a_new_infos)
    {
        m_distance_to_go = a_new_infos->distance_to_go;
        m_homing_sequences_done = a_new_infos->homing_sequences_done;
    };

private:
    uint16_t m_speed;       // mm/s
    uint16_t m_max_accel;   // mm/s²
    uint16_t m_max_current; // mA

public:
    StepperMotor(uint16_t speed, uint16_t accel, uint16_t max_current)
      : m_speed(speed)
      , m_max_accel(accel)
      , m_max_current(max_current)
    {
        m_stepper_mode = StepperMode::DISABLE;
        m_target_position = 0;
        m_homing_sequences_done = 0;
    };

    explicit StepperMotor()
      : StepperMotor(100, 100, 100)
    {
    }
    // void publish(float a_speed, float a_angle);

    void setSpeed(uint16_t a_speed)
    {
        m_speed = a_speed;
    };
    void setEnable(StepperMode a_new_mode)
    {
        m_stepper_mode = a_new_mode;
    };

    uint16_t getAccel()
    {
        return m_max_accel;
    };
    uint16_t getSpeed()
    {
        return m_speed;
    };
    uint16_t getMaxCurrent()
    {
        return m_max_current;
    };
    uint16_t getTargetPosition()
    {
        return m_target_position;
    };
    StepperMode getStepperMode()
    {
        return m_stepper_mode;
    };
    bool movmentDone()
    {
        return m_distance_to_go == 0;
    };
};

class StepperElevator : public StepperMotor
{
private:
    // float m_;
    bool m_homing_done;
    volatile uint8_t m_homing_sequences_done;
    int16_t m_max_height_mm;
    int16_t m_min_height_mm = -150; // mm (negative value because the elevator goes down)

public:
    void updateElevatorInfos(krabi_msgs::msg::InfosStepper::SharedPtr a_new_infos)
    {
        m_homing_done = a_new_infos->homing_switch_on;
        m_homing_sequences_done = a_new_infos->homing_sequences_done;
        updateInfos(a_new_infos);
    };
    bool doHoming()
    {
        auto l_nb_homing_sequences_done = m_homing_sequences_done;
        m_stepper_mode = StepperMode::HOMING;

        usleep(0.2e6);
        goToPosition(0); // Rest position, after homing is done

        auto l_timeout_homing_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (l_nb_homing_sequences_done == m_homing_sequences_done)
        {
            if (std::chrono::steady_clock::now() > l_timeout_homing_deadline)
            {
                // Failed to home in time
                return false;
            }
            // wait for homing to be done
            usleep(10000);
        }
        return true;
    };
    bool homingDone()
    {
        return m_homing_done;
    };
    bool goToPosition(int16_t a_distance_in_mm);

    StepperElevator(uint16_t a_max_height_mm)
      : StepperMotor()
    {
        m_max_height_mm = a_max_height_mm;
    };

    StepperElevator(uint16_t speed,
                    uint16_t accel,
                    uint16_t max_current,
                    int16_t a_max_height_mm,
                    int16_t a_min_height_mm)
      : StepperMotor(speed, accel, max_current)
    {
        m_max_height_mm = a_max_height_mm;
        m_min_height_mm = a_min_height_mm;
    };
};