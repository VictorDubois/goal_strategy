#include "krabilib/strategie/aireDeDepose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#include "krabilib/strategie/strategieV2.h"
//#define VITESSE_LINEAIRE_MAX 100
#endif

#ifdef QTGUI
#include <QDebug>
#endif

#define QTGUI

#define qDebug() std::cout

AireDeDepose::AireDeDepose()
{
}

AireDeDepose::AireDeDepose(Position goalPosition, Owner us_or_them) : AireDeDepose(goalPosition, goalPosition, us_or_them)
{
}


AireDeDepose::AireDeDepose(Position goalPosition, Position area_center, Owner us_or_them)
  : MediumLevelAction(goalPosition)
{
  m_goal_position=goalPosition;
  m_us_or_them=us_or_them;
  m_area_center=area_center;
}

AireDeDepose::~AireDeDepose()
{
}

Owner AireDeDepose::getOwner()
{
    return m_us_or_them;
}

Etape::EtapeType AireDeDepose::getType()
{
    return Etape::AIRE_DE_DEPOSE;
}

void AireDeDepose::addPlant(Plant added_plant)
{
    m_stock.push_back(added_plant);
}

void AireDeDepose::addPlants(std::vector<Plant> added_plants)
{
    m_stock.insert(m_stock.end(), added_plants.begin(), added_plants.end());
}

std::vector<Plant> AireDeDepose::getPlants()
{
    return m_stock;
}

unsigned int AireDeDepose::getNumberOfPlants()
{
    return m_stock.size();
}

Position AireDeDepose::getAreaCenter()
{
    return m_area_center;
}

int AireDeDepose::update()
{
    return 0;
}
