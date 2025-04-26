#pragma once
#include "rclcpp/rclcpp.hpp"
#include "servomotor.h"

class AX12 : public Servomotor
{
private:
public:
    AX12(float speed, float angle)
      : Servomotor(speed, angle) {};
    AX12()
      : AX12(0, 128)
    {
    }
};
