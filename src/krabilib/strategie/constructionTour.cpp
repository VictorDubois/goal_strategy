#include "krabilib/strategie/constructionTour.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

EmplacementTour::EmplacementTour()
{
}

EmplacementTour::EmplacementTour(Pose goalPose)
  : EmplacementTour(goalPose, goalPose)
{
}

EmplacementTour::EmplacementTour(Pose goalPose, Pose area_center)
  : MediumLevelAction(goalPose.getPosition())
{
    m_goal_pose = goalPose;
    m_area_center = area_center;
}

EmplacementTour::~EmplacementTour()
{
}

Etape::EtapeType EmplacementTour::getType()
{
    return Etape::EMPLACEMENT_TOUR;
}

Pose EmplacementTour::getAreaCenter()
{
    return m_area_center;
}

Pose EmplacementTour::getGoalPose()
{
    return m_goal_pose;
}

Position EmplacementTour::getGoalPosition()
{
    return m_goal_pose.getPosition();
}

int EmplacementTour::update()
{
    return 0;
}
