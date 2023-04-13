#pragma once

#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/pileGateau.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

enum Owner
{
    us,
    them
};

class Assiette : public MediumLevelAction
{
public:
    Assiette();

    Assiette(Position position, Position assiette_center, Owner us_or_them);
    Assiette(Position position, Owner us_or_them);

    ~Assiette();

    int update();

    Etape::EtapeType getType();

    Owner getOwner();

    void addGateau(CoucheGateau flavor);
    std::vector<CoucheGateau> getGateaux();

    unsigned int getNumberOFGateaux();

    Position getAssietteCenter();

protected:
    Position position;
    Position position_depart;
    Position m_assiette_center;
    Owner m_us_or_them;

    std::vector<CoucheGateau> m_stock;
};
