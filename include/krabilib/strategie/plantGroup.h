#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE


class PlantGroup : public MediumLevelAction
{
public:
    PlantGroup();

    PlantGroup(Position position);

    ~PlantGroup();


    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};
