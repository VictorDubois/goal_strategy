#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

class Vitrine : public MediumLevelAction
{
public:
    Vitrine();

    Vitrine(Position position);

    ~Vitrine();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};
