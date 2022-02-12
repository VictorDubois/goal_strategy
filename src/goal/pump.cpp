#include "../include/goal_strategy/pump.h"

Pump::Pump(bool enable, bool release)
  : m_enable(enable)
  , m_release(release)
{
}

void Pump::setPumping(bool a_enable)
{
    m_enable = a_enable;
    if (m_enable)
    {
        m_release = false;
    }
}

void Pump::release()
{
    m_release = true;
}

bool Pump::getPumping()
{
    return m_enable;
}

bool Pump::getRelease()
{
    return m_release;
}
