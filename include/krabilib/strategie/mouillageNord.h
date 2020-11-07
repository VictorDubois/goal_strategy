#ifndef MouillageNord_H
#define MouillageNord_H

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
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
