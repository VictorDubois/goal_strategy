#include "krabilib/strategie/graal.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

Graal::Graal()
{
}

Graal::Graal(Pose goalPose)
  : Graal(goalPose, goalPose)
{
}

Graal::Graal(Pose goalPose, Pose area_center)
  : MediumLevelAction(goalPose.getPosition())
{
    m_goal_pose = goalPose;
    m_area_center = area_center;
}

Graal::~Graal()
{
}

Etape::EtapeType Graal::getType()
{
    return Etape::GRAAL;
}

Pose Graal::getAreaCenter()
{
    return m_area_center;
}

Pose Graal::getGoalPose()
{
    return m_goal_pose;
}

Position Graal::getGoalPosition()
{
    return m_goal_pose.getPosition();
}

int Graal::update()
{
    return 0;
}
