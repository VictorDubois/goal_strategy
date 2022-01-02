#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

class Hexagon_plat : public MediumLevelAction
{
public:
    Hexagon_plat();

    Hexagon_plat(Position position, bool isGreen);

    ~Hexagon_plat();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
    bool isGreen;
};
