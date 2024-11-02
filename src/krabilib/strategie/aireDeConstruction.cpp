#include "krabilib/strategie/aireDeConstruction.h"
#include "krabilib/strategie/stockDeMatierePremiere.h" // for plateforme definition
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#include "krabilib/strategie/strategieV2.h"
//#define VITESSE_LINEAIRE_MAX 100
#endif


#define qDebug() std::cout

AireDeConstruction::AireDeConstruction()
{
}

AireDeConstruction::AireDeConstruction(Pose goalPose, Owner us_or_them, AireSize a_size) 
    : AireDeConstruction(goalPose, goalPose, us_or_them, a_size)
{
}


AireDeConstruction::AireDeConstruction(Pose goalPose, Pose area_center, Owner us_or_them, AireSize a_size)
  : MediumLevelAction(goalPose.getPosition())
{
  m_size = a_size;
  m_goal_pose=goalPose;
  m_us_or_them=us_or_them;
  m_area_center=area_center;
}

AireDeConstruction::~AireDeConstruction()
{
}

Owner AireDeConstruction::getOwner()
{
    return m_us_or_them;
}

Etape::EtapeType AireDeConstruction::getType()
{
    return Etape::AIRE_DE_CONSTRUCTION;
}

void AireDeConstruction::addPlateforme(Plateforme added_plateforme)
{
    m_stock.push_back(added_plateforme);
}

std::vector<Plateforme> AireDeConstruction::getPlateformes()
{
    return m_stock;
}

Pose AireDeConstruction::getAreaCenter()
{
    return m_area_center;
}

Pose AireDeConstruction::getGoalPose()
{
    return m_goal_pose;
}

Position AireDeConstruction::getGoalPosition()
{
    return m_goal_pose.getPosition();
}

int AireDeConstruction::update()
{
    return 0;
}