#ifndef COUPE2019_H
#define COUPE2019_H

#define ETAPE_GARAGE 1

#define NOMBRE_ETAPES 30

#include "Krabi/strategie/abeille.h"
#include "Krabi/strategie/accelerator.h"
#include "Krabi/strategie/etape.h"
#include "Krabi/strategie/goldenium.h"
#include "Krabi/strategie/reservoirEau.h"
#include "Krabi/strategie/strategiev3.h"
#include <geometry_msgs/Point.h>

#include <geometry_msgs/Pose.h>

class Coupe2019 : public StrategieV3
{
public:
    /** @brief Constructeur de la stratégie *
     * @param isBlue le côté de départ */
    Coupe2019(bool isYellow, std::vector<geometry_msgs::Pose> etapes);
    Coupe2019(bool isYellow, std::vector<geometry_msgs::Point> etapes);

    void setRemainingTime(float seconds_left);
    void setGoodMouillage(Etape::EtapeType good_mouillage);

    std::vector<geometry_msgs::Point> getPositions();

private:
    //	/** Nombre d'étapes dans le graph */
    //	int nombreEtapes = NOMBRE_ETAPES;

    //	/** Numéro de l'étape où le robot va se cacher à la fin */
    //	int numeroEtapeGarage = ETAPE_GARAGE;

    /** @brief update du score d'une étape */
    int getScoreEtape(int i);

    float remainingTime;
    Etape::EtapeType good_mouillage;

    // Attractor* m_attractors;
};

#endif // COUPE2019_H
