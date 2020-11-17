#include "goal_strategy/coupe2020.h"
#include "krabilib/strategie/mancheAAir.h"
#include "krabilib/strategie/mouillageNord.h"
#include "krabilib/strategie/mouillageSud.h"
#include "krabilib/strategie/phare.h"
#include "krabilib/strategie/port.h"
#include <cmath>
#include <iostream>
#include "ros/ros.h"
#include <cstdlib>


²

MissionPlanningCoupe2021::MissionPlanningCoupe2021(const bool isBlue) : StrategieV3(isBlue) {
	//Initialisation des tableaux d'étapes
	this->numeroEtapeGarage = Eta;
	tableauEtapesTotal = Etape::initTableauEtapeTotal(etapes_as_points.size());
	
	Position position1 = Position(etapes_as_points[0]);
	//positionFromPoint(etapes_as_points[0]);
	std::cout << "posX = " << position1.getX() << std::endl;
    bool isBlue = false;
    int start = Etape::makeEtape(Position(0, 0, isBlue), Etape::DEPART);
    int pp1 = Etape::makeEtape(Position(0, 0, isBlue), Etape::POINT_PASSAGE);
    int pp2 = Etape::makeEtape(Position(1, 0, isBlue), Etape::POINT_PASSAGE);
    int goal = Etape::makeEtape(Position(0, 1, isBlue), Etape::ABEILLE);
	Etape::get(pp1)->addVoisins(start);
	Etape::get(pp1)->addVoisins(pp2);
	Etape::get(pp2)->addVoisins(goal);

    this->nombreEtapes = Etape::getTotalEtapes();

    // Lancer Dijkstra
    startDijkstra();
}

std::vector<geometry_msgs::Point> Coupe2021::getPositions()
{
    std::vector<geometry_msgs::Point> l_points;
    for (int positionID = 0; positionID < nombreEtapes; positionID++)
    {
        l_points.push_back(tableauEtapesTotal[positionID]->getPosition().getPoint());
    }

    return l_points;
}

void Coupe2021::setRemainingTime(float a_seconds_left)
{
    remainingTime = a_seconds_left;
}

void Coupe2021::setGoodMouillage(Etape::EtapeType a_good_mouillage)
{
    good_mouillage = a_good_mouillage;
}

int Coupe2021::getScoreEtape(int i)
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
