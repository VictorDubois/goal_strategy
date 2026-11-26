#include "krabilib/strategie/carrierePierres.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

CarrierePierres::CarrierePierres()
{
}

CarrierePierres::CarrierePierres(Pose goalPose)
  : CarrierePierres(goalPose, goalPose)
{
}

CarrierePierres::CarrierePierres(Pose goalPose, Pose area_center)
  : MediumLevelAction(goalPose.getPosition())
{
    m_goal_pose = goalPose;
    m_area_center = area_center;
}

CarrierePierres::~CarrierePierres()
{
}

Etape::EtapeType CarrierePierres::getType()
{
    return Etape::CARRIERE_PIERRES;
}

Pose CarrierePierres::getAreaCenter()
{
    return m_area_center;
}

Pose CarrierePierres::getGoalPose()
{
    return m_goal_pose;
}

Position CarrierePierres::getGoalPosition()
{
    return m_goal_pose.getPosition();
}

int CarrierePierres::update()
{
    return 0;
}

std::vector<Pierre> CarrierePierres::getPierres()
{
    return m_pierres;
}

void CarrierePierres::retirePierres()
{
    m_pierres.clear();
}
