#pragma once

#define NOMBRE_ETAPES 50

#include "krabilib/strategie/abeille.h"
#include "krabilib/strategie/accelerator.h"
#include "krabilib/strategie/assiette.h"
#include "krabilib/strategie/aireDeDepose.h"
#include "krabilib/strategie/bouee.h"
#include "krabilib/strategie/carreFouille.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/galerie.h"
#include "krabilib/strategie/goldenium.h"
#include "krabilib/strategie/hexagon_plat.h"
#include "krabilib/strategie/pileGateau.h"
#include "krabilib/strategie/plantGroup.h"
#include "krabilib/strategie/reservoirEau.h"
#include "krabilib/strategie/statuette.h"
#include "krabilib/strategie/strategiev3.h"
#include "krabilib/strategie/vitrine.h"
#include "krabilib/strategie/stockDeMatierePremiere.h"
#include "krabilib/strategie/aireDeConstruction.h"
#include <geometry_msgs/msg/point.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

enum StartingPosition2025
{
    FRONT_START,
    COTE_START,
    PAMI_START
};


class Coupe2025 : public StrategieV3
{
public:
    Coupe2025(bool isYellow, StartingPosition2025 starting_position);
    void debugEtapes(visualization_msgs::msg::MarkerArray& ma);
    void etape_type_to_marker(visualization_msgs::msg::Marker& m, Etape* e);

    void catchStatuette(){}; // 2022 only
    void dropStatuette(){};  // 2022 only

    void grabGateau(Etape* e){}; //2023 only
    int dropGateau(Etape* e){}; //2023 only

    void grabPlant(Etape* e) {}; // 2024 only
    int dropPlant(Etape* e) {}; // 2024 only

    void grabPlateformes(Etape* e); //2025 only
    int dropPlateformes(Etape* e);  //2025 only

    void dropBanner(Etape* e); //2025 only

    Position positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left);

    AireDeConstruction* getBestAreaForFunny(); // 2024 and 2025

    std::vector<Plateforme> getStock();

private:
    int getScoreEtape(int i);

    uint32_t m_seq;

    std::vector<Plateforme> m_stock;
};
