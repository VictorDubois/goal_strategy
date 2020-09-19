#ifndef PORT_H
#define PORT_H

#include "Krabi/position.h"
#include "Krabi/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "Krabi/command.h"
#endif // STANDALONE_STRATEGIE

class Port : public MediumLevelAction
{
public:
    Port();

    Port(Position position);

    ~Port();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};

#endif // PORT_H
