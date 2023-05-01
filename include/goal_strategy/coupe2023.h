#pragma once

#define NOMBRE_ETAPES 50

#include "krabilib/strategie/abeille.h"
#include "krabilib/strategie/accelerator.h"
#include "krabilib/strategie/assiette.h"
#include "krabilib/strategie/bouee.h"
#include "krabilib/strategie/carreFouille.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/galerie.h"
#include "krabilib/strategie/goldenium.h"
#include "krabilib/strategie/hexagon_plat.h"
#include "krabilib/strategie/pileGateau.h"
#include "krabilib/strategie/reservoirEau.h"
#include "krabilib/strategie/statuette.h"
#include "krabilib/strategie/strategiev3.h"
#include "krabilib/strategie/vitrine.h"
#include <geometry_msgs/Point.h>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Coupe2023 : public StrategieV3
{
public:
    Coupe2023(bool isYellow);
    void debugEtapes(visualization_msgs::MarkerArray& ma);
    void etape_type_to_marker(visualization_msgs::Marker& m, Etape* e);

    void catchStatuette(){}; // 2022 only
    void dropStatuette(){};  // 2022 only

    void grabGateau(Etape* e);
    int dropGateau(Etape* e);

    Position positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left);

private:
    int getScoreEtape(int i);

    uint32_t m_seq;

    std::vector<CoucheGateau> m_stock;
};
