#include "../include/goal_strategy/pump.h"

Pump::Pump(bool enable)
  : m_enable(enable)
{
}

void Pump::setPumping(bool a_enable)
{
    m_enable = a_enable;
}

bool Pump::getPumping()
{
    return m_enable;
}
