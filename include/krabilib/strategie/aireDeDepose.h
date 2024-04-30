#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/plantGroup.h"
#include "krabilib/strategie/assiette.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE


class AireDeDepose : public MediumLevelAction
{
public:
    AireDeDepose();

    AireDeDepose(Position goalPosition, Position area_center, Owner us_or_them);
    AireDeDepose(Position goalPosition, Owner us_or_them);

    ~AireDeDepose();

    int update();

    Etape::EtapeType getType();

    Owner getOwner();


    void addPlant(Plant added_plant);

    std::vector<Plant> getPlants();

    unsigned int getNumberOfPlants();

    Position getAreaCenter();

protected:
    Position m_goal_position;
    Position m_area_center;
    Owner m_us_or_them;
    std::vector<Plant> m_stock;
};
