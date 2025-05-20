#pragma once

#define NOMBRE_ETAPES 50

#include "krabilib/strategie/abeille.h"
#include "krabilib/strategie/accelerator.h"
#include "krabilib/strategie/aireDeDepose.h"
#include "krabilib/strategie/assiette.h"
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
#include <geometry_msgs/msg/point.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

enum StartingPosition
{
    SOLAR_PANEL,
    CENTER,
    PAMI
};

class Coupe2024 : public StrategieV3
{
public:
    Coupe2024(bool isYellow, StartingPosition starting_position);
    void debugEtapes(visualization_msgs::msg::MarkerArray& ma);
    void etape_type_to_marker(visualization_msgs::msg::Marker& m, Etape* e);

    void catchStatuette() {}; // 2022 only
    void dropStatuette() {};  // 2022 only

    void grabGateau(Etape* /*e*/) {}; // 2023 only
    int dropGateau(Etape* /*e*/) {};  // 2023 only

    void grabPlant(Etape* e);
    int dropPlant(Etape* e);

    Position positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left);

    AireDeDepose* getBestAreaForFunny();

    std::vector<Plant> getStock();

private:
    int getScoreEtape(int i);

    uint32_t m_seq;

    std::vector<Plant> m_stock;
};
