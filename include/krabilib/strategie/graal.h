#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

/** @brief Graal pickup spot (salle du trone). The Graal is then carried on top of the robot (level 4, roof above 30 cm) until the end of the match: there is no drop etape. */
class Graal : public MediumLevelAction
{
public:
    Graal();

    Graal(Pose goalPose, Pose area_center);
    Graal(Pose goalPose);

    ~Graal();

    int update();

    Etape::EtapeType getType();

    Pose getAreaCenter();
    Position getGoalPosition();
    Pose getGoalPose();

protected:
    Pose m_goal_pose;
    Pose m_area_center;
};
