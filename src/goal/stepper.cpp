#include "../include/goal_strategy/stepper.h"

bool StepperElevator::goToPosition(int16_t a_distance_in_mm)
{
    m_stepper_mode = StepperMode::POSITION;
    if (m_max_height_mm < a_distance_in_mm)
    {
        m_target_position = m_max_height_mm;
        return true; // error!
    }
    else if (a_distance_in_mm < m_min_height_mm)
    {
        m_target_position = m_min_height_mm;
        return true; // error!
    }
    m_target_position = a_distance_in_mm;
    return false;
}
