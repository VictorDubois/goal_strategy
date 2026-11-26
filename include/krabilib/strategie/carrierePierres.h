#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/pierre.h"
#include <vector>

/** @brief Carriere de pierres: 3 pierres stacked horizontally. Etape where the robot grabs pierres. */
class CarrierePierres : public MediumLevelAction
{
public:
    CarrierePierres();

    CarrierePierres(Pose goalPose, Pose area_center);
    CarrierePierres(Pose goalPose);

    ~CarrierePierres();

    int update();

    Etape::EtapeType getType();

    Pose getAreaCenter();
    Position getGoalPosition();
    Pose getGoalPose();

    /** @brief Pierres still in the carriere (3 at the start of the match) */
    std::vector<Pierre> getPierres();
    void retirePierres();

protected:
    Pose m_goal_pose;
    Pose m_area_center;
    std::vector<Pierre> m_pierres{ NOMBRE_PIERRES_PAR_CARRIERE, Pierre() };
};
