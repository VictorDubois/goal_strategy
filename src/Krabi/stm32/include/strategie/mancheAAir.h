#ifndef MANCHE_A_AIR_H
#define MANCHE_A_AIR_H

#include "Krabi/position.h"
#include "Krabi/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "command.h"
#else
#include "Krabi/asservissement/command.h"
#endif // STANDALONE_STRATEGIE

class MancheAAir : public MediumLevelAction
{
public:
    MancheAAir();

    MancheAAir(Position position);

    ~MancheAAir();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};

#endif // MANCHE_A_AIR_H
