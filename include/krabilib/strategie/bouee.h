#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

class Bouee : public MediumLevelAction
{
public:
    Bouee();

    Bouee(Position position, bool isGreen);

    ~Bouee();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
    bool isGreen;
};
