#ifndef MANCHE_A_AIR_H
#define MANCHE_A_AIR_H

#include "position.h"
#include "mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "command.h"
#else
#include "commandSSA.h"
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
