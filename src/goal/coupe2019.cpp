#include <iostream>
#include "goal_strategy/coupe2019.h"
#include <cmath>
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
void cart_to_polar(int posX, int posY, float& theta, float& distance) {
	theta = ((180./M_PI) * atan2((float) posX, (float) posY));
	distance = sqrt((float)(posX * posX + posY * posY))/1000.f;

	// fix angular ambiguity
	if (posY < 0) {
		theta += 180;
	}

	#ifdef DEBUG_cart_to_polar
		std::cout << "posX = " << posX << "posY = " <<  posY<< "theta = " << theta << ", distance = " << distance << std::endl;
		int posXafter =(int)1000* distance*cos(theta * M_PI/180.f);
        	int posYafter =(int)1000* distance*sin(theta * M_PI/180.f);
	        std::cout << "posXafter = " << posXafter << ", posYafter = " << posYafter << std::endl;
	#endif
}

Coupe2019::Coupe2019(const bool isYellow, const std::vector<geometry_msgs::Pose> etapesAsPoses) : StrategieV3(isYellow) {
	std::vector<geometry_msgs::Point> etapesAsPoints;
	for (auto pose: etapesAsPoses) {
		etapesAsPoints.push_back(pose.position);
	}	
	Coupe2019(isYellow, etapesAsPoints);
}

Coupe2019::Coupe2019(const bool isYellow, const std::vector<geometry_msgs::Point> etapes_as_points) : StrategieV3(isYellow) {	
	//Initialisation des tableaux d'étapes

    //tableauEtapesTotal = Etape::initTableauEtapeTotal(etapes_as_points.size());//NOMBRE_ETAPES);//new Etape*[NOMBRE_ETAPES];

	// Création des étapes
	// Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions

	// Initialisation in simulator in initKrabi.cpp
    //Initialisation des tableaux d'étapes
    this->numeroEtapeGarage = ETAPE_GARAGE;
    tableauEtapesTotal = Etape::initTableauEtapeTotal(NOMBRE_ETAPES);//new Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions

    // Initialisation in simulator in initKrabi.cpp
    int main_port = Etape::makeEtape(Position(200, 800, isYellow), Etape::DEPART); // départ au fond de la zone de départ

    int lighthouse = Etape::makeEtape(new Goldenium(Position(301,  200, isYellow)));

    int out_of_lighthouse = Etape::makeEtape(Position(640, 400, isYellow));

    int out_of_main_port = Etape::makeEtape(Position(700, 800, isYellow));

    Etape::get(main_port)->addVoisins(out_of_main_port);
    Etape::get(lighthouse)->addVoisins(out_of_lighthouse);
    Etape::get(out_of_main_port)->addVoisins(out_of_lighthouse);

    int first_air = Etape::makeEtape(new Accelerator(Position(230, 1800, isYellow)));
    int second_air = Etape::makeEtape(new Accelerator(Position(635, 1800, isYellow)));
    int out_of_air = Etape::makeEtape(Position(430, 1400, isYellow));

    Etape::get(first_air)->addVoisins(out_of_air);
    Etape::get(second_air)->addVoisins(out_of_air);

    int south = Etape::makeEtape(new Abeille(Position(150, 1250, isYellow)));
    int north = Etape::makeEtape(new Abeille(Position(150, 320, isYellow)));

    Etape::get(south)->addVoisins(out_of_air);
    Etape::get(north)->addVoisins(out_of_lighthouse);
    Etape::get(north)->addVoisins(lighthouse);

    /** Points de passage **/
    int waypoint_south = Etape::makeEtape(Position(640, 1300, isYellow));
    int waypoint_out_of_enemy_port = Etape::makeEtape(Position(1100, 1400, isYellow));
    Etape::get(out_of_air)->addVoisins(waypoint_south);
    Etape::get(out_of_main_port)->addVoisins(waypoint_south);
    Etape::get(waypoint_out_of_enemy_port)->addVoisins(waypoint_south);

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

int Coupe2019::getScoreEtape(int i)
{

	switch (this->tableauEtapesTotal[i]->getEtapeType())
	{
		/*case Etape::TYPE_ACTION:
			return NB_POINTS_ACTION; */


		// A faire : remplacer la priorite par le nombre de points obtenables a l'etape

		case Etape::DEPART :
			return 0;
		case Etape::POINT_PASSAGE :
			return 0;
		case Etape::ABEILLE :
            return 10;
        case Etape::GOLDENIUM :
            return 100;
        case Etape::ACCELERATOR :
            return 100;
		case Etape::RESERVOIR_EAU :
			return 0; //@TODO new type that is only worth points after accelerator done

		default :
			return 1; /* DEBUG (0 sinon) */
	}
}

