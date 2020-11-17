#include "goal_strategy/coupe2019.h"
#include "krabilib/strategie/mancheAAir.h"
#include "krabilib/strategie/mouillageNord.h"
#include "krabilib/strategie/mouillageSud.h"
#include "krabilib/strategie/phare.h"
#include "krabilib/strategie/port.h"
#include <cmath>
#include <iostream>
#include "ros/ros.h"
#include <cstdlib>

Coupe2019::Coupe2019(const bool isYellow, const std::vector<geometry_msgs::Pose> etapesAsPoses)
  : StrategieV3(isYellow)
{
    std::vector<geometry_msgs::Point> etapesAsPoints;
    for (auto pose : etapesAsPoses)
    {
        etapesAsPoints.push_back(pose.position);
    }
    Coupe2019(isYellow, etapesAsPoints);
}

//Coupe2019::Coupe2019(bool isYellow, Attractor* attractors, unsigned int sizeofAttractors) : StrategieV3(isYellow)
Coupe2019::Coupe2019(const bool isYellow, const std::vector<geometry_msgs::Point> etapes_as_points) : StrategieV3(isYellow) {
	//Initialisation des tableaux d'étapes
	this->numeroEtapeGarage = ETAPE_GARAGE;
	tableauEtapesTotal = Etape::initTableauEtapeTotal(etapes_as_points.size());//NOMBRE_ETAPES);//new Etape*[NOMBRE_ETAPES];

	// Création des étapes
	// Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions

    bool isBlue = false;
    int start = Etape::makeEtape(Position(Distance(0), Distance(0)), Etape::DEPART); // départ au fond de la zone de départ
    int pp1 = Etape::makeEtape(Position(Distance(0), Distance(0)), Etape::POINT_PASSAGE); // départ au fond de la zone de départ
    int pp2 = Etape::makeEtape(Position(Distance(1), Distance(0)), Etape::POINT_PASSAGE); // départ au fond de la zone de départ
    int goal = Etape::makeEtape(Position(Distance(0), Distance(1)), Etape::ABEILLE); // départ au fond de la zone de départ
	Etape::get(pp1)->addVoisins(start);
	Etape::get(pp1)->addVoisins(pp2);
	Etape::get(pp2)->addVoisins(goal);

    this->nombreEtapes = Etape::getTotalEtapes();

    // Lancer Dijkstra
    startDijkstra();
}

std::vector<geometry_msgs::Point> Coupe2019::getPositions()
{
    std::vector<geometry_msgs::Point> l_points;
    for (int positionID = 0; positionID < nombreEtapes; positionID++)
    {
        l_points.push_back(tableauEtapesTotal[positionID]->getPosition());
    }

    return l_points;
}

void Coupe2019::setRemainingTime(float a_seconds_left)
{
    remainingTime = a_seconds_left;
}

void Coupe2019::setGoodMouillage(Etape::EtapeType a_good_mouillage)
{
    good_mouillage = a_good_mouillage;
}

int Coupe2019::getScoreEtape(int i)
{
    int l_score = 0;
    switch (this->tableauEtapesTotal[i]->getEtapeType())
    {
    case Etape::DEPART:
        l_score = 0;
        break;
    case Etape::POINT_PASSAGE:
        l_score = 0;
        break;
    case Etape::PHARE:
        l_score = 200;
        break;
    case Etape::MANCHE_A_AIR:
        l_score = 100;
        break;
    case Etape::PORT:
        l_score = 10;
        break;
    case Etape::MOUILLAGE_NORD:
        l_score = 0;
        if (good_mouillage == Etape::MOUILLAGE_NORD && remainingTime < 10.f)
        {
            l_score = 100;
        }
        break;
    case Etape::MOUILLAGE_SUD:
        l_score = 0;
        if (good_mouillage == Etape::MOUILLAGE_SUD && remainingTime < 10.f)
        {
            l_score = 100;
        }
        break;

    default:
        return 0;
    }
    return l_score;
}
