#pragma once


#define NOMBRE_ETAPES 50

#include "krabilib/strategie/abeille.h"
#include "krabilib/strategie/accelerator.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/goldenium.h"
#include "krabilib/strategie/reservoirEau.h"
#include "krabilib/strategie/strategiev3.h"
#include <geometry_msgs/Point.h>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>



class Coupe2021 : public StrategieV3
{
public:
    Coupe2021(bool isYellow);

    void setGoodMouillage(Etape::EtapeType good_mouillage);

    void debugEtapes(visualization_msgs::MarkerArray& ma);


private:
    int getScoreEtape(int i);

    Etape::EtapeType m_good_mouillage;
    int m_south_id;
    int m_north_id;
    uint32_t m_seq;

};