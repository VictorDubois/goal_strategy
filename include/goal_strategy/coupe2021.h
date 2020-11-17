#pragma once

#include "krabilib/strategie/abeille.h"
#include "krabilib/strategie/accelerator.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/goldenium.h"
#include "krabilib/strategie/reservoirEau.h"
#include "krabilib/strategie/strategiev3.h"
#include <geometry_msgs/Point.h>

#include <geometry_msgs/Pose.h>

enum class Etape2021{
        DEPART,
        PHARE,
        MANCHE_A_AIR,
        PORT,
        LECTURE_GIROUETTE
        MOUILLAGE_NORD,
        MOUILLAGE_SUD,
}

class Coupe2021 : public StrategieV3
{
public:
    /** @brief Constructeur de la stratégie *
     * @param isBlue le côté de départ */
    MissionPlanningCoupe2021(bool isBlue);

    void setRemainingTime(float seconds_left);

    std::vector<geometry_msgs::Point> getPositions();

private:
    /** @brief update du score d'une étape */
    int getScoreEtape(int i);

    float remainingTime;
    Etape::EtapeType good_mouillage;
};
