#ifndef ACCELERATOR_H
#define ACCELERATOR_H

#include "Krabi/position.h"
#include "Krabi/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "Krabi/asservissement/command.h"
#endif // STANDALONE_STRATEGIE

class Accelerator : public MediumLevelAction
{
public:
    Accelerator();

    Accelerator(Position position);

    ~Accelerator();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};

#endif // ACCELERATOR_H
