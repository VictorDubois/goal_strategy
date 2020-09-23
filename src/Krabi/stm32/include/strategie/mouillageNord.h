#ifndef MouillageNord_H
#define MouillageNord_H

#include "Krabi/position.h"
#include "Krabi/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "Krabi/command.h"
#endif // STANDALONE_STRATEGIE

class MouillageNord : public MediumLevelAction
{
public:
    MouillageNord();

    MouillageNord(Position position);

    ~MouillageNord();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};

#endif // MouillageNord_H
