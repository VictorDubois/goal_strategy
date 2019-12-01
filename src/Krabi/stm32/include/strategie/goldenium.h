#ifndef GOLDENIUM_H
#define GOLDENIUM_H

#include "Krabi/position.h"
#include "Krabi/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "Krabi/asservissement/command.h"
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
