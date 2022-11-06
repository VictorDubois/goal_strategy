#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

enum Owner
{
    us,
    them
};

class Assiette : public MediumLevelAction
{
public:
    Assiette();

    Assiette(Position position, Owner us_or_them);

    ~Assiette();

    int update();

    Etape::EtapeType getType();

    Owner getOwner();

protected:
    Position position;
    Position position_depart;
    Owner us_or_them;
};
