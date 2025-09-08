#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

enum Orientation
{
    us,
    them,
    both_long_side,
    both_small_side
};

class Caisse
{
    Caisse(Orientation a_orientation)
      : m_orientation(a_orientation) {};

public:
    Orientation getOrientation();

private:
    Orientation m_orientation;
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
