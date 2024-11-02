#pragma once

#include "krabilib/position.h"
#include "krabilib/pose.h"
#include "krabilib/strategie/mediumLevelAction.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

class Plateforme
{
    public:
        Plateforme(){};
        Plateforme(Pose start_pose){};

    private:
        int m_current_level;
        Pose m_pose;
        Pose m_start_pose;
};

class StockDeMatierePremiere : public MediumLevelAction
{
public:
    StockDeMatierePremiere();

    StockDeMatierePremiere(Pose pose);

    ~StockDeMatierePremiere();

    Etape::EtapeType getType();

    std::vector<Plateforme> getPlateformes();

    int update();

    void EmptyStockDeMatierePremiere();

    Pose getStockCenter(){return m_start_pose;}

    
protected:
    Position m_position;
    Pose m_start_pose;
    std::vector<Plateforme> m_plateformes;
};
