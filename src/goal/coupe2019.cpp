#include "goal_strategy/coupe2019.h"
#include "Krabi/strategie/mancheAAir.h"
#include "Krabi/strategie/mouillageNord.h"
#include "Krabi/strategie/mouillageSud.h"
#include "Krabi/strategie/phare.h"
#include "Krabi/strategie/port.h"
#include <cmath>
#include <iostream>
#define NB_NEURONS 360
#define PI 3.14159265
#include "ros/ros.h"
#include <cstdlib>

#ifdef QTGUI
#include <QDebug>
#endif

/**
 * Convert a cartesian position to a polar one
 * @param posX the X position, in mm
 * @param posY the Y position, in mm
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
//#define DEBUG_cart_to_polar
void cart_to_polar(int posX, int posY, float& theta, float& distance)
{
    theta = ((180. / M_PI) * atan2((float)posX, (float)posY));
    distance = sqrt((float)(posX * posX + posY * posY)) / 1000.f;

    // fix angular ambiguity
    if (posY < 0)
    {
        theta += 180;
    }

#ifdef DEBUG_cart_to_polar
    std::cout << "posX = " << posX << "posY = " << posY << "theta = " << theta
              << ", distance = " << distance << std::endl;
    int posXafter = (int)1000 * distance * cos(theta * M_PI / 180.f);
    int posYafter = (int)1000 * distance * sin(theta * M_PI / 180.f);
    std::cout << "posXafter = " << posXafter << ", posYafter = " << posYafter << std::endl;
#endif
}

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

Coupe2019::Coupe2019(const bool isYellow, const std::vector<geometry_msgs::Point> etapes_as_points)
  : StrategieV3(isYellow)
{
    remainingTime = 100;
    good_mouillage = Etape::EtapeType::DEPART; // No mouillage is good yet

    // Initialisation des tableaux d'étapes

    // tableauEtapesTotal =
    // Etape::initTableauEtapeTotal(etapes_as_points.size());//NOMBRE_ETAPES);//new
    // Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions

    // Initialisation in simulator in initKrabi.cpp
    // Initialisation des tableaux d'étapes
    this->numeroEtapeGarage = ETAPE_GARAGE;
    tableauEtapesTotal = Etape::initTableauEtapeTotal(NOMBRE_ETAPES); // new Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions
    int main_port = Etape::makeEtape(Position(200, 800, isYellow),
                                     Etape::DEPART); // départ au fond de la zone de départ

    int lighthouse = Etape::makeEtape(new Phare(Position(450, 200, isYellow)));

    int out_of_lighthouse = Etape::makeEtape(Position(450, 300, isYellow));

    int out_of_main_port = Etape::makeEtape(Position(700, 800, isYellow));

    Etape::get(main_port)->addVoisins(out_of_main_port);
    Etape::get(lighthouse)->addVoisins(out_of_lighthouse);
    Etape::get(out_of_main_port)->addVoisins(out_of_lighthouse);

    int first_air = Etape::makeEtape(new MancheAAir(Position(230, 1800, isYellow)));
    int second_air = Etape::makeEtape(new MancheAAir(Position(635, 1800, isYellow)));
    int out_of_first_air = Etape::makeEtape(Position(230, 1600, isYellow));
    int out_of_second_air = Etape::makeEtape(Position(635, 1600, isYellow));

    Etape::get(out_of_second_air)->addVoisins(out_of_first_air);
    Etape::get(first_air)->addVoisins(out_of_first_air);
    Etape::get(second_air)->addVoisins(out_of_second_air);

    int south = Etape::makeEtape(new MouillageSud(Position(150, 1250, isYellow)));
    int north = Etape::makeEtape(new MouillageNord(Position(150, 320, isYellow)));

    Etape::get(south)->addVoisins(out_of_second_air);
    Etape::get(south)->addVoisins(out_of_first_air);
    Etape::get(north)->addVoisins(out_of_lighthouse);
    Etape::get(north)->addVoisins(lighthouse);

    int push_south_bouees = Etape::makeEtape(new Port(Position(350, 1000, isYellow)));
    int push_north_bouees = Etape::makeEtape(new Port(Position(350, 600, isYellow)));
    Etape::get(out_of_lighthouse)->addVoisins(push_north_bouees);

    // Points de passage
    int waypoint_south = Etape::makeEtape(Position(640, 1300, isYellow));
    int waypoint_out_of_enemy_small_port = Etape::makeEtape(Position(1100, 1400, isYellow));
    Etape::get(south)->addVoisins(waypoint_south);
    Etape::get(out_of_first_air)->addVoisins(waypoint_south);
    Etape::get(out_of_second_air)->addVoisins(waypoint_south);
    Etape::get(out_of_second_air)->addVoisins(waypoint_out_of_enemy_small_port);
    Etape::get(out_of_main_port)->addVoisins(waypoint_south);
    Etape::get(out_of_main_port)->addVoisins(waypoint_out_of_enemy_small_port);
    Etape::get(waypoint_out_of_enemy_small_port)->addVoisins(waypoint_south);

    int waypoint_middle_ports = Etape::makeEtape(Position(1500, 1400, isYellow));
    int waypoint_out_of_our_small_port = Etape::makeEtape(Position(1800, 1400, isYellow));
    Etape::get(waypoint_out_of_enemy_small_port)->addVoisins(waypoint_middle_ports);
    Etape::get(waypoint_out_of_our_small_port)->addVoisins(waypoint_middle_ports);

    int waypoint_out_of_push_south_bouees = Etape::makeEtape(Position(480, 1300, isYellow));
    Etape::get(waypoint_out_of_push_south_bouees)->addVoisins(push_south_bouees);

    Etape::get(waypoint_out_of_push_south_bouees)->addVoisins(waypoint_south);
    Etape::get(waypoint_out_of_push_south_bouees)->addVoisins(out_of_second_air);

    int our_small_port = Etape::makeEtape(new Port(Position(1800, 1750, isYellow)));
    Etape::get(our_small_port)->addVoisins(waypoint_out_of_our_small_port);
#ifdef QTGUI
    qDebug() << Etape::getTotalEtapes();
#endif

    // Certaines actions d'étapes ne finnissent pas là où elles ont commencé :
    // Clapets:
    // Etape::get(4)->setNumeroEtapeFinAction(43); //Clapet notre côté vers notre bord

    this->nombreEtapes = Etape::getTotalEtapes();

    // Lancer Dijkstra
    startDijkstra();
}

std::vector<geometry_msgs::Point> Coupe2019::getPositions()
{
    std::vector<geometry_msgs::Point> l_points;
    for (int positionID = 0; positionID < nombreEtapes; positionID++)
    {
        l_points.push_back(tableauEtapesTotal[positionID]->getPosition().getPoint());
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
        /*case Etape::TYPE_ACTION:
                return NB_POINTS_ACTION; */

        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::DEPART:
        l_score = 0;
        break;
    case Etape::POINT_PASSAGE:
        l_score = 0;
        break;
    case Etape::PHARE:
        l_score = 100;
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
