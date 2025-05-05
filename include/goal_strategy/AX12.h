#pragma once
#include "rclcpp/rclcpp.hpp"
#include "servomotor.h"

class AX12 : public Servomotor
{
private:
    int m_mode = 1;
    uint8_t m_temperature_limit = 65; // °C
    uint8_t m_current_limit = 255;    // ??
    uint8_t m_max_accel = 255;        // ??

public:
    AX12(float speed, float angle)
      : Servomotor(speed, angle) {};
    AX12()
      : AX12(0, 128)
    {
    }

    uint8_t getTemperatureLimit()
    {
        return m_temperature_limit;
    }

    uint8_t getCurrentLimit()
    {
        return m_current_limit;
    }

    uint8_t getMode()
    {
        return m_mode;
    }

    uint8_t getTorqueEnable()
    {
        return getSpeed() != 0;
    }

    uint8_t getMaxAccel()
    {
        return m_max_accel;
    }
};
