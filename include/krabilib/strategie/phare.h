#ifndef PHARE_H
#define PHARE_H

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
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
