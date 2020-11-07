#ifndef GOLDENIUM_H
#define GOLDENIUM_H

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/asservissement/command.h"
#endif // STANDALONE_STRATEGIE

class Goldenium : public MediumLevelAction
{
public:
    Goldenium();

    Goldenium(Position position);

    ~Goldenium();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};

#endif // GOLDENIUM_H
