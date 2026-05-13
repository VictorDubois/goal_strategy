#pragma once
#include "servomotor.h"
#include <stdint.h>

enum AX12_CURRENT : uint8_t
{
    OFF = 0,
    OPEN_CLAW = 40,
    MAX_CURRENT = 100
};

class AX12 : public Servomotor
{
private:
    int m_mode = 1;
    uint8_t m_temperature_limit = 65;                    // °C
    uint8_t m_current_limit = AX12_CURRENT::MAX_CURRENT; // ??
    uint8_t m_max_accel = 100;                           // ??
    int16_t m_present_position = 0;
    uint8_t m_present_temperature = 0;
    uint16_t m_present_current = 0;
    uint8_t m_moving = 0;

public:
    AX12(float speed, float angle)
      : Servomotor(speed, angle) {};
    AX12()
      : AX12(0, 128)
    {
    }

    void updateInfos(int16_t a_current_position,
                     uint8_t a_present_temperature,
                     uint16_t a_present_current,
                     uint8_t a_moving)
    {
        m_present_position = a_current_position;
        m_present_temperature = a_present_temperature;
        m_present_current = a_present_current;
        m_moving = a_moving;
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

    void setCurrentLimit(uint8_t a_current_limit)
    {
        m_current_limit = a_current_limit;
    }
};
