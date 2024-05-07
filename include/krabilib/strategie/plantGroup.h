#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

enum PlantType
{
    STURDY,
    FRAGILE
};

class Plant
{
    public:
        Plant(){};
        Plant(PlantType plant_type, Position start_position){};

    private:
        PlantType m_plant_type;
        Position m_position;
        Position m_start_position;
};

class PlantGroup : public MediumLevelAction
{
public:
    PlantGroup();

    PlantGroup(Position position);

    ~PlantGroup();

    Etape::EtapeType getType();

    std::vector<Plant> getPlants();

    int update();

    void EmptyPlantGroup();

    
protected:
    Position m_position;
    Position m_start_position;
    std::vector<Plant> m_plants;
};
