#include "krabilib/strategie/thermometre.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#define qDebug() std::cout

Thermometre::Thermometre()
{
}

Thermometre::Thermometre(Pose goalPose)
  : Thermometre(goalPose, goalPose)
{
}

Thermometre::Thermometre(Pose goalPose, Pose area_center)
  : MediumLevelAction(goalPose.getPosition())
{
    m_goal_pose = goalPose;
    m_area_center = area_center;
}

Thermometre::~Thermometre()
{
}

Etape::EtapeType Thermometre::getType()
{
    return Etape::THERMOMETRE;
}

Pose Thermometre::getAreaCenter()
{
    return m_area_center;
}

Pose Thermometre::getGoalPose()
{
    return m_goal_pose;
}

Position Thermometre::getGoalPosition()
{
    return m_goal_pose.getPosition();
}

int Thermometre::update()
{
    return 0;
}