#include "krabilib/strategie/stockDeMatierePremiere.h"
#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

#define qDebug() std::cout

StockDeMatierePremiere::StockDeMatierePremiere()
{
    Plateforme plateforme;
    m_plateformes.push_back(plateforme);
}

StockDeMatierePremiere::StockDeMatierePremiere(Pose goalPose)
  : MediumLevelAction(goalPose.getPosition())
{
    goalPosition = this->goalPosition;
    m_start_pose = goalPose;

    Plateforme plateforme;
    m_plateformes.push_back(plateforme);
}

StockDeMatierePremiere::~StockDeMatierePremiere()
{
}

Etape::EtapeType StockDeMatierePremiere::getType()
{
    return Etape::STOCK_MATIERE_PREMIERE;
}

std::vector<Plateforme> StockDeMatierePremiere::getPlateformes()
{
    return m_plateformes;
}

int StockDeMatierePremiere::update()
{
    return 0;
}

void StockDeMatierePremiere::EmptyStockDeMatierePremiere()
{
    m_plateformes.clear();
}