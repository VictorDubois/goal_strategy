#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

class Port : public MediumLevelAction
{
public:
    Port();

    Port(Position position);

    ~Port();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};