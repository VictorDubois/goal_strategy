#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

/** @brief Emplacement de tour (4 per team, 200 mm diameter): 1 pierre verticale. */
class EmplacementTour : public MediumLevelAction
{
public:
    EmplacementTour();

    EmplacementTour(Pose goalPose, Pose area_center);
    EmplacementTour(Pose goalPose);

    ~EmplacementTour();

    int update();

    Etape::EtapeType getType();

    Pose getAreaCenter();
    Position getGoalPosition();
    Pose getGoalPose();

protected:
    Pose m_goal_pose;
    Pose m_area_center;
};
