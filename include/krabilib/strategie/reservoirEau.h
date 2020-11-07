#ifndef RESERVOIR_EAU_H
#define RESERVOIR_EAU_H

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/asservissement/command.h"
#endif // STANDALONE_STRATEGIE

class ReservoirEau : public MediumLevelAction
{
public:
    ReservoirEau();

    ReservoirEau(Position position);

    ~ReservoirEau();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};

#endif // RESERVOIR_EAU_H
