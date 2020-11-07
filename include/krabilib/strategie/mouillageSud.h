#ifndef MouillageSud_H
#define MouillageSud_H

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

class MouillageSud : public MediumLevelAction
{
public:
    MouillageSud();

    MouillageSud(Position position);

    ~MouillageSud();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};

#endif // MouillageSud_H
