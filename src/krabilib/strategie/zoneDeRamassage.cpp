#include "krabilib/strategie/zoneDeRamassage.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#define qDebug() std::cout

ZoneDeRamassage::ZoneDeRamassage()
{
}

ZoneDeRamassage::ZoneDeRamassage(Pose goalPose)
  : ZoneDeRamassage(goalPose, goalPose)
{
}

ZoneDeRamassage::ZoneDeRamassage(Pose goalPose, Pose area_center)
  : MediumLevelAction(goalPose.getPosition())
{
    m_goal_pose = goalPose;
    m_area_center = area_center;
}

ZoneDeRamassage::~ZoneDeRamassage()
{
}

Etape::EtapeType ZoneDeRamassage::getType()
{
    return Etape::ZONE_DE_RAMASSAGE;
}

Pose ZoneDeRamassage::getAreaCenter()
{
    return m_area_center;
}

Pose ZoneDeRamassage::getGoalPose()
{
    return m_goal_pose;
}

Position ZoneDeRamassage::getGoalPosition()
{
    return m_goal_pose.getPosition();
}

int ZoneDeRamassage::update()
{
    return 0;
}

std::vector<Caisse> ZoneDeRamassage::getCaisses()
{
    return m_stock;
}