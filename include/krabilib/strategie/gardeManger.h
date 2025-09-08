#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "zoneDeRamassage.h" // pour def Caisse

class GardeManger : public MediumLevelAction
{
public:
    GardeManger();

    GardeManger(Pose goalPose, Pose area_center);
    GardeManger(Pose goalPose);

    ~GardeManger();

    int update();

    Etape::EtapeType getType();

    std::vector<Caisse> getCaisses();
    void addCaisse(Caisse);

    Pose getAreaCenter();
    Position getGoalPosition();
    Pose getGoalPose();

protected:
    Pose m_goal_pose;
    Pose m_area_center;
    std::vector<Caisse> m_stock;
};
