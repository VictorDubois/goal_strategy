#include "../include/goal_strategy/stepper.h"

bool StepperElevator::goToPosition(int16_t a_distance_in_mm)
{
    m_stepper_mode = StepperMode::POSITION;
    if (a_max_height_mm < a_distance_in_mm)
    {
        m_target_position = a_max_height_mm;
        return true; // error!
    }
    else if (a_distance_in_mm < 0)
    {
        m_target_position = 0;
        return true; // error!
    }
    m_target_position = a_distance_in_mm;
}
