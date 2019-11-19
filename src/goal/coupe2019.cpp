#include <iostream>
#include "goal_strategy/coupe2019.h"
#include <cmath>
#define NB_NEURONS 360
#define PI 3.14159265

#ifdef QTGUI
	#include <QDebug>
#endif

unsigned int get_idx_of_max_2 (float * vector, size_t len = NB_NEURONS) {
        unsigned int curr_max = 0, i;

        for (i = 0; i < len; i += 1) {
                if (vector[i] > vector[curr_max])
                        curr_max = i;
        }

        return curr_max;
}

Position Coupe2019::positionFromAttractor(int attractorId) {
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
}

Coupe2019::Coupe2019(bool isYellow, Attractor* attractors, unsigned int sizeofAttractors) : StrategieV3(isYellow)
{
	m_attractors = attractors;
	
	//Initialisation des tableaux d'étapes
	this->numeroEtapeGarage = ETAPE_GARAGE;
	tableauEtapesTotal = Etape::initTableauEtapeTotal(NOMBRE_ETAPES);//new Etape*[NOMBRE_ETAPES];

	// Création des étapes
	// Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions

	// Initialisation in simulator in initKrabi.cpp
	//int start = Etape::makeEtape(positionFromAttractor(0), Etape::DEPART); // départ au fond de la zone de départ
	//int start = Etape::makeEtape(Position(200, 350, true), Etape::DEPART); // départ au fond de la zone de départ


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
	
	/** Points de passage **/
	int waypoint_behind_backhole = Etape::makeEtape(positionFromAttractor(attractorId++));
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
