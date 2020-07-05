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
	//m_attractors = attractors;
	
	//Initialisation des tableaux d'étapes
	this->numeroEtapeGarage = ETAPE_GARAGE;
	tableauEtapesTotal = Etape::initTableauEtapeTotal(etapes_as_points.size());//NOMBRE_ETAPES);//new Etape*[NOMBRE_ETAPES];

	// Création des étapes
	// Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions

	// Initialisation in simulator in initKrabi.cpp
	//int etapeId = 0;
	//int start = Etape::makeEtape(positionFromAttractor(0), Etape::DEPART); // départ au fond de la zone de départ
	//int start = Etape::makeEtape(positionFromPoint(etapes_as_points[++etapeId]), Etape::DEPART); // départ au fond de la zone de départ
	Position position1 = Position(etapes_as_points[0]);
	//positionFromPoint(etapes_as_points[0]);
	std::cout << "posX = " << position1.getX() << std::endl;
	//Etape::makeEtape(position1, Etape::DEPART); // départ au fond de la zone de départ
    bool isBlue = false;
    int start = Etape::makeEtape(Position(0, 0, isBlue), Etape::DEPART); // départ au fond de la zone de départ
    int pp1 = Etape::makeEtape(Position(0, 200, isBlue), Etape::POINT_PASSAGE); // départ au fond de la zone de départ
    int pp2 = Etape::makeEtape(Position(200, 200, isBlue), Etape::POINT_PASSAGE); // départ au fond de la zone de départ
    int goal = Etape::makeEtape(Position(200, 0, isBlue), Etape::ABEILLE); // départ au fond de la zone de départ
	Etape::get(pp1)->addVoisins(start);
	Etape::get(pp1)->addVoisins(pp2);
	Etape::get(pp2)->addVoisins(goal);
	//Etape::get(pp2)->addVoisins(start);

/*
	// COLORS
	int mid_red = Etape::makeEtape(Position(0,0, true), Etape::DEPART);
	int attractorId = 0;
	int out_of_red = Etape::makeEtape(positionFromAttractor(attractorId++));
	Etape::get(mid_red)->addVoisins(out_of_red);// out of start
	
	int out_of_green = Etape::makeEtape(positionFromAttractor(attractorId++));
	int mid_green = Etape::makeEtape(new Abeille(positionFromAttractor(attractorId++)));
	Etape::get(mid_green)->addVoisins(out_of_green);// push greenium
	Etape::get(out_of_red)->addVoisins(out_of_green);// From red to green
	
	//int mid_blue = Etape::makeEtape(new Abeille(positionFromAttractor(attractorId++)));
	//int out_of_blue = Etape::makeEtape(positionFromAttractor(attractorId++));
	//Etape::get(mid_blue)->addVoisins(out_of_blue);// push blue
	//Etape::get(out_of_blue)->addVoisins(out_of_green);// from green to blue
	
	// ACCELERATOR
	int out_of_top_of_accel = Etape::makeEtape(positionFromAttractor(attractorId++));
	int in_front_of_top_of_accel = Etape::makeEtape(new Accelerator(positionFromAttractor(attractorId++)));
	Etape::get(in_front_of_top_of_accel)->addVoisins(out_of_top_of_accel);// accel
	Etape::get(out_of_red)->addVoisins(out_of_top_of_accel);// red to accel
	
	// GOLDENIUM
	int out_of_goldenium = Etape::makeEtape(positionFromAttractor(attractorId++));
	int in_front_of_goldenium = Etape::makeEtape(new Goldenium(positionFromAttractor(attractorId++)));
	Etape::get(out_of_goldenium)->addVoisins(in_front_of_goldenium);// goldenium
	Etape::get(out_of_top_of_accel)->addVoisins(out_of_goldenium);// goldenium to accel
*/	
	/** Points de passage **/
/*	int waypoint_behind_backhole = Etape::makeEtape(positionFromAttractor(attractorId++));
	Etape::get(out_of_green)->addVoisins(waypoint_behind_backhole);// green to waypoint
	Etape::get(out_of_red)->addVoisins(waypoint_behind_backhole);// red to waypoint
	Etape::get(out_of_top_of_accel)->addVoisins(waypoint_behind_backhole);// accel to waypoint
	
	//int second_waypoint_behind_backhole = Etape::makeEtape(positionFromAttractor(attractorId++));
	//Etape::get(out_of_top_of_accel)->addVoisins(second_waypoint_behind_backhole);// accel to 2waypoint
	//Etape::get(waypoint_behind_backhole)->addVoisins(second_waypoint_behind_backhole);// waypoint to 2waypoint

	// Reservoir eau proche
	//int reservoirProche = Etape::makeEtape(new ReservoirEau(Position(220, 840, true)));

	for (auto i = 0; i < attractorId; i++) {
		positionFromAttractor(i);
	}
*/
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

