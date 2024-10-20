#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/plantGroup.h"
#include "krabilib/strategie/assiette.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE


class AireDeConstruction : public MediumLevelAction
{
public:
    AireDeConstruction();

    AireDeConstruction(Position goalPosition, Position area_center, Owner us_or_them);
    AireDeConstruction(Position goalPosition, Owner us_or_them);

    ~AireDeConstruction();

    int update();
    
    Etape::EtapeType getType();

    Owner getOwner();


    void addPlateforme(Plant added_plant);

    std::vector<Plant> getPlateformes();

    Position getAreaCenter();
    Position getGoalPosition();

protected:
    Position m_goal_position;
    Position m_area_center;
    Owner m_us_or_them;
    std::vector<Plant> m_stock;
};
