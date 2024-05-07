#include "krabilib/strategie/plantGroup.h"
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

PlantGroup::PlantGroup()
{
    Plant plant;
    m_plants.push_back(plant);
    m_plants.push_back(plant);
    m_plants.push_back(plant);
    m_plants.push_back(plant);
    m_plants.push_back(plant);
    m_plants.push_back(plant);
}

PlantGroup::PlantGroup(Position goalPosition)
  : MediumLevelAction(goalPosition)
{
    goalPosition = this->goalPosition;
    m_start_position = goalPosition;
    
    Plant plant;
    m_plants.push_back(plant);
    m_plants.push_back(plant);
    m_plants.push_back(plant);
    m_plants.push_back(plant);
    m_plants.push_back(plant);
    m_plants.push_back(plant);
}

PlantGroup::~PlantGroup()
{
}


Etape::EtapeType PlantGroup::getType()
{
    return Etape::PLANT_GROUP;
}

std::vector<Plant> PlantGroup::getPlants()
{
    return m_plants;
}


int PlantGroup::update()
{
    return 0;
}

void PlantGroup::EmptyPlantGroup()
{
    m_plants.clear();
}