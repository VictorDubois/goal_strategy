#ifndef RESERVOIR_EAU_H
#define RESERVOIR_EAU_H

#include "Krabi/position.h"
#include "Krabi/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "Krabi/asservissement/command.h"
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
