#include "krabilib/strategie/gardeManger.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/zoneDeRamassage.h" // for caisse definition

#define qDebug() std::cout

GardeManger::GardeManger()
{
}

GardeManger::GardeManger(Pose goalPose)
  : GardeManger(goalPose, goalPose)
{
}

GardeManger::GardeManger(Pose goalPose, Pose area_center)
  : MediumLevelAction(goalPose.getPosition())
{
    m_goal_pose = goalPose;
    m_area_center = area_center;
}

GardeManger::~GardeManger()
{
}

Etape::EtapeType GardeManger::getType()
{
    return Etape::GARDE_MANGER;
}

void GardeManger::addCaisse(Caisse added_caisse)
{
    m_stock.push_back(added_caisse);
}

std::vector<Caisse> GardeManger::getCaisses()
{
    return m_stock;
}

Pose GardeManger::getAreaCenter()
{
    return m_area_center;
}

Pose GardeManger::getGoalPose()
{
    return m_goal_pose;
}

Position GardeManger::getGoalPosition()
{
    return m_goal_pose.getPosition();
}

int GardeManger::update()
{
    return 0;
}