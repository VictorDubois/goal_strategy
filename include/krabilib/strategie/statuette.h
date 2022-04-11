#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

class Statuette : public MediumLevelAction
{
public:
    Statuette();

    Statuette(Position position);

    ~Statuette();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};
