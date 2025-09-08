#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

class Thermometre : public MediumLevelAction
{
public:
    Thermometre();

    Thermometre(Pose goalPose, Pose area_center);
    Thermometre(Pose goalPose);

    ~Thermometre();

    int update();

    Etape::EtapeType getType();

    Pose getAreaCenter();
    Position getGoalPosition();
    Pose getGoalPose();

protected:
    Pose m_goal_pose;
    Pose m_area_center;
};
