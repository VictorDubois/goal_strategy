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

    AireDeDepose(Position goalPosition, Position area_center, Owner us_or_them, bool is_starting_position);
    AireDeDepose(Position goalPosition, Owner us_or_them, bool is_starting_position);

    ~AireDeDepose();

    int update();
    
    bool isStartingPosition();

    Etape::EtapeType getType();

    Owner getOwner();


    void addPlant(Plant added_plant);
    void addPlants(std::vector<Plant> added_plants);

    std::vector<Plant> getPlants();

    unsigned int getNumberOfPlants();

    Position getAreaCenter();
    Position getGoalPosition();

    bool isValidAreaForFunny();

protected:
    Position m_goal_position;
    Position m_area_center;
    Owner m_us_or_them;
    std::vector<Plant> m_stock;
    bool m_is_starting_position;
};
