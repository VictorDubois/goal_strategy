#pragma once

#define NOMBRE_ETAPES 50

#include "krabilib/strategie/abeille.h"
#include "krabilib/strategie/accelerator.h"
#include "krabilib/strategie/bouee.h"
#include "krabilib/strategie/carreFouille.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/galerie.h"
#include "krabilib/strategie/goldenium.h"
#include "krabilib/strategie/hexagon_plat.h"
#include "krabilib/strategie/reservoirEau.h"
#include "krabilib/strategie/statuette.h"
#include "krabilib/strategie/strategiev3.h"
#include "krabilib/strategie/vitrine.h"
#include <geometry_msgs/msg/point.h>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class Coupe2022 : public StrategieV3
{
public:
    Coupe2022(bool isYellow);
    void debugEtapes(visualization_msgs::msg::MarkerArray& ma);
    void etape_type_to_marker(visualization_msgs::msg::Marker& m, const Etape::EtapeType& e);

    void catchStatuette();
    void dropStatuette();
    void grabGateau(Etape*){}; // 2023 only
    void dropGateau(Etape*){}; // 2023 only

    Position positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left);

private:
    int getScoreEtape(int i);

    bool statuette_held;

    uint32_t m_seq;
};
