#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

enum CoucheGateau
{
    glacage_rose,
    creme_jaune,
    genoise_marron
};

class PileGateau : public MediumLevelAction
{
public:
    PileGateau();

    PileGateau(Position position, CoucheGateau type_couche);

    ~PileGateau();

    int update();

    Etape::EtapeType getType();

    const CoucheGateau getTypeCouche();

protected:
    Position position;
    Position position_depart;
    CoucheGateau type_couche;
};
