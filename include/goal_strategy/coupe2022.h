#pragma once

#define NOMBRE_ETAPES 50

#include "krabilib/strategie/abeille.h"
#include "krabilib/strategie/accelerator.h"
#include "krabilib/strategie/bouee.h"
#include "krabilib/strategie/hexagon_plat.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/goldenium.h"
#include "krabilib/strategie/reservoirEau.h"
#include "krabilib/strategie/strategiev3.h"
#include <geometry_msgs/Point.h>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Coupe2022 : public StrategieV3
{
public:
    Coupe2022(bool isYellow);
    void debugEtapes(visualization_msgs::MarkerArray& ma);

    Position positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left);

private:
    int getScoreEtape(int i);

    uint32_t m_seq;
};
