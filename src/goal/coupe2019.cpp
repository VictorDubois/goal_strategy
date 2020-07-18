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

/*Position Coupe2019::positionFromAttractor(int attractorId) {
	unsigned int angle = get_idx_of_max_2(m_attractors[attractorId].position, NB_NEURONS);
	float distance = m_attractors[attractorId].position[angle];
	std::cout << "AttractorId = " << attractorId << ", angle = " << angle << ", distance = " << distance << std::endl;
	int posX, posY;
	posX =(int)1000* distance*cos(angle * PI/180);
	posY =(int)1000* distance*sin(angle * PI/180);
	std::cout << "posX = " << posX << ", posY = " << posY << std::endl;
	bool color;
	Position newPos = Position(posX, posY, color);
	return newPos;
}*/

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
void createEtapes(bool is_blue) {

	std::vector<std::pair<int, int> > positionsCart;
	std::vector<std::pair<float, float> > positionsPolar;

	// first: longueur (vers l'autre couleur)
	// second: profondeur (vers le publique)
    std::pair<int, int> positionDepart = std::make_pair(300, 450);
    int angleDepart = -90;//degrees
    positionsCart.push_back(std::make_pair(800, 450));//OK          out of red
	positionsCart.push_back(std::make_pair(800, 750));//OK          out of green
	positionsCart.push_back(std::make_pair(300, 750));//OK          mid green
	positionsCart.push_back(std::make_pair(1700, 450));//           out of top of accel
	positionsCart.push_back(std::make_pair(1700, 200));//           in front of top of accel
	if (is_blue) { //BLUE
		positionsCart.push_back(std::make_pair(2250, 450));//            out of goldenium
		positionsCart.push_back(std::make_pair(2250, 200));//            in front of goldenium
	} else { // YELLOW
		positionsCart.push_back(std::make_pair(2230, 450));//            out of goldenium
		positionsCart.push_back(std::make_pair(2230, 200));//            in front of goldenium
	}
	positionsCart.push_back(std::make_pair(1500, 800));//          waypoint behind backhole


	// Create polar position from cartesian positions
	for (auto position: positionsCart) {
		float distance = 0;
		float angle = 0;
		cart_to_polar(position.first - positionDepart.first, position.second - positionDepart.second, angle, distance);
		angle += angleDepart;
		positionsPolar.push_back(std::make_pair(distance, angle));
	}

	// Revert for the other color
	if (is_blue) { // BLUE
		printf("Reversing position because of color choice\n");
		fflush(stdout);
		for (auto& position: positionsPolar) {
			std::cout << "position, before: " << position.second << std::endl;
			position.second = - position.second;
			std::cout << "after: " << position.second << std::endl;
		}
		fflush(stdout);
	}
	else {
		printf("Keeping positions\n");
		fflush(stdout);
	}

	std::cout << "There are " << positionsPolar.size() << " positionPolar, " << positionsCart.size() << " positionCart" << std::endl;
	fflush(stdout);
}

Coupe2019::Coupe2019(const bool isYellow, const std::vector<geometry_msgs::Pose> etapesAsPoses) : StrategieV3(isYellow) {
	std::vector<geometry_msgs::Point> etapesAsPoints;
	for (auto pose: etapesAsPoses) {
		etapesAsPoints.push_back(pose.position);
	}	
	Coupe2019(isYellow, etapesAsPoints);
}

//Coupe2019::Coupe2019(bool isYellow, Attractor* attractors, unsigned int sizeofAttractors) : StrategieV3(isYellow)
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
    int main_port = Etape::makeEtape(Position(200, 800, true), Etape::DEPART); // départ au fond de la zone de départ

    int lighthouse = Etape::makeEtape(Position(301,  200, true));

    int out_of_lighthouse = Etape::makeEtape(Position(640, 400, true));

    int out_of_main_port = Etape::makeEtape(Position(700, 800, true));

    Etape::get(main_port)->addVoisins(out_of_main_port);
    Etape::get(lighthouse)->addVoisins(out_of_lighthouse);
    Etape::get(out_of_main_port)->addVoisins(out_of_lighthouse);

    int first_air = Etape::makeEtape(new Abeille(Position(230, 1800, true)));
    int second_air = Etape::makeEtape(new Abeille(Position(635, 1800, true)));
    int out_of_air = Etape::makeEtape(Position(430, 1400, true));

    Etape::get(first_air)->addVoisins(out_of_air);
    Etape::get(second_air)->addVoisins(out_of_air);

    int south = Etape::makeEtape(new Abeille(Position(150, 1250, true)));
    int north = Etape::makeEtape(new Abeille(Position(150, 320, true)));

    Etape::get(south)->addVoisins(out_of_air);
    Etape::get(north)->addVoisins(out_of_lighthouse);
    Etape::get(north)->addVoisins(lighthouse);


    /** Points de passage **/
    int waypoint_south = Etape::makeEtape(Position(640, 1300, true));
    int waypoint_out_of_enemy_port = Etape::makeEtape(Position(1100, 1400, true));
    Etape::get(out_of_air)->addVoisins(waypoint_south);// green to waypoint
    Etape::get(out_of_main_port)->addVoisins(waypoint_south);// red to waypoint
    Etape::get(waypoint_out_of_enemy_port)->addVoisins(waypoint_south);// red to waypoint


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
			return 100;
		case Etape::RESERVOIR_EAU :
			return 0; //@TODO new type that is only worth points after accelerator done


//		case Etape::ZONE_CONSTRUCTION : {
//			if (benne->getIsBenneEmpty()) {
//				return 1;
//			}
//			else {
//				return 1000;
//			}
//		}
//
//		case Etape::CABINE :
//			return 400;
//
//		case Etape::CUBE_DEBUT :
//			return 500;
//
//		case Etape::DUNE : {
//			if( benne->getIsBenneEmpty() ) {
//				return 400;
//			}
//			else {
//				return 1;
//			}
//		}

		default :
			return 1; /* DEBUG (0 sinon) */
	}
}

