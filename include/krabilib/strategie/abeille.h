#ifndef ABEILLE_H
#define ABEILLE_H

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/asservissement/command.h"
#endif // STANDALONE_STRATEGIE

class Abeille : public MediumLevelAction
{
public:
    Abeille();

    Abeille(Position position);

    ~Abeille();

    int update();

    Etape::EtapeType getType();

protected:
    Position position;
    Position position_depart;
};

#endif // ABEILLE_H
