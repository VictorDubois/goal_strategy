#ifndef PHARE_H
#define PHARE_H

#include "Krabi/position.h"
#include "Krabi/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "Krabi/command.h"
#endif // STANDALONE_STRATEGIE

class Phare : public MediumLevelAction
{
public:
    Phare();

    Phare(Position position);

    ~Phare();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};

#endif // PHARE_H
