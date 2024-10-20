#include "krabilib/strategie/aireDeConstruction.h"
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

AireDeConstruction::AireDeConstruction(Position goalPosition, Owner us_or_them) 
    : AireDeConstruction(goalPosition, goalPosition, us_or_them)
{
}


AireDeConstruction::AireDeConstruction(Position goalPosition, Position area_center, Owner us_or_them)
  : MediumLevelAction(goalPosition)
{
  m_goal_position=goalPosition;
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

void AireDeConstruction::addPlateforme(Plant added_plant)
{
    m_stock.push_back(added_plant);
}

std::vector<Plant> AireDeConstruction::getPlateformes()
{
    return m_stock;
}

Position AireDeConstruction::getAreaCenter()
{
    return m_area_center;
}

Position AireDeConstruction::getGoalPosition()
{
    return m_goal_position;
}

int AireDeConstruction::update()
{
    return 0;
}