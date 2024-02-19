#pragma once

#define NOMBRE_ETAPES 50

#include "krabilib/strategie/abeille.h"
#include "krabilib/strategie/accelerator.h"
#include "krabilib/strategie/bouee.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/goldenium.h"
#include "krabilib/strategie/reservoirEau.h"
#include "krabilib/strategie/strategiev3.h"
#include <geometry_msgs/msg/point.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class Coupe2021 : public StrategieV3
{
public:
    Coupe2021(bool isYellow);
    void setGoodMouillage(Etape::EtapeType good_mouillage);
    Etape::EtapeType getGoodMouillage();
    void debugEtapes(visualization_msgs::msg::MarkerArray& ma);

    Position positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left);

private:
    int getScoreEtape(int i);

    Etape::EtapeType m_good_mouillage;
    int m_south_id;
    int m_north_id;
    uint32_t m_seq;
};
