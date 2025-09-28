#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

enum OrientationCaisse
{
    unknown,
    unknown_team_side,
    our_side,
    other_side,
    common_long_side,
    common_small_side_wtf,
};

class Caisse
{
public:
    Caisse(OrientationCaisse a_orientation)
      : m_orientation(a_orientation) {};
    Caisse()
      : m_orientation(OrientationCaisse::unknown_team_side) {};

    OrientationCaisse getOrientation();

private:
    OrientationCaisse m_orientation;
};

class ZoneDeRamassage : public MediumLevelAction
{
public:
    ZoneDeRamassage();

    ZoneDeRamassage(Pose goalPose, Pose area_center);
    ZoneDeRamassage(Pose goalPose);

    ~ZoneDeRamassage();

    int update();

    Etape::EtapeType getType();

    std::vector<Caisse> getCaisses();

    Pose getAreaCenter();
    Position getGoalPosition();
    Pose getGoalPose();

protected:
    Pose m_goal_pose;
    Pose m_area_center;
    std::vector<Caisse> m_stock;
};
