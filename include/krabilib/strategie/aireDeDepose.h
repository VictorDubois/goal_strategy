#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/pileGateau.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

enum Owner
{
    us,
    them
};

class AireDeDepose : public MediumLevelAction
{
public:
    AireDeDepose();

    AireDeDepose(Position goalPosition, Position assiette_center, Owner us_or_them);
    AireDeDepose(Position goalPosition, Owner us_or_them);

    ~AireDeDepose();

    //int update();

    Etape::EtapeType getType();

    Owner getOwner();

    void addPlants(PlantGroup added_plant_group);
    void addPlant(Plant added_plant);

    std::vector<CoucheGateau> getGateaux();

    unsigned int getNumberOfPlant();

    Position getAreaCenter();

protected:
    Position m_goal_position;
    Position m_area_center;
    Owner m_us_or_them;

    std::vector<Plant> m_stock;
};
