#include "krabilib/strategie/constructionMur.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

EmplacementMur::EmplacementMur()
{
}

EmplacementMur::EmplacementMur(Pose goalPose)
  : EmplacementMur(goalPose, goalPose)
{
}

EmplacementMur::EmplacementMur(Pose goalPose, Pose area_center)
  : MediumLevelAction(goalPose.getPosition())
{
    m_goal_pose = goalPose;
    m_area_center = area_center;
}

EmplacementMur::~EmplacementMur()
{
}

Etape::EtapeType EmplacementMur::getType()
{
    return Etape::EMPLACEMENT_MUR;
}

Pose EmplacementMur::getAreaCenter()
{
    return m_area_center;
}

Pose EmplacementMur::getGoalPose()
{
    return m_goal_pose;
}

Position EmplacementMur::getGoalPosition()
{
    return m_goal_pose.getPosition();
}

int EmplacementMur::update()
{
    return 0;
}

bool EmplacementMur::isPorte() const
{
    return m_porte;
}

void EmplacementMur::setPorte(bool a_porte)
{
    m_porte = a_porte;
}
