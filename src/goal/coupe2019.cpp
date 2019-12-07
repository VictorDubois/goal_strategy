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

unsigned int get_idx_of_max_2 (float * vector, size_t len = NB_NEURONS) {
        unsigned int curr_max = 0, i;

        for (i = 0; i < len; i += 1) {
                if (vector[i] > vector[curr_max])
                        curr_max = i;
        }

        return curr_max;
}

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

Position Coupe2019::positionFromPose(geometry_msgs::Pose pose) {
	return this->positionFromPoint(pose.position);
}

Position Coupe2019::positionFromPoint(geometry_msgs::Point point) {
	std::cout << "posX = " << point.x << ", posY = " << point.y << std::endl;
	bool color = true;
	Position newPos = Position(point.x, point.y, color);
	return newPos;
}

geometry_msgs::Pose Coupe2019::poseFromPosition(Position position) {
	geometry_msgs::Point point = pointFromPosition(position);
	geometry_msgs::Pose pose;
	pose.position = point;
	return pose;
}

geometry_msgs::Point Coupe2019::pointFromPosition(Position position) {
	geometry_msgs::Point point;
	point.x = position.getX();
	point.y = position.getY();
	point.z = 0;
	std::cout << "posX = " << point.x << ", posY = " << point.y << std::endl;
	return point;
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
	Position position1 = positionFromPoint(etapes_as_points[0]);
	//positionFromPoint(etapes_as_points[0]);
	std::cout << "posX = " << position1.getX() << std::endl;
	//Etape::makeEtape(position1, Etape::DEPART); // départ au fond de la zone de départ
	int start = Etape::makeEtape(Position(3, 100, true), Etape::DEPART); // départ au fond de la zone de départ
	int pp1 = Etape::makeEtape(Position(300, 350, true), Etape::POINT_PASSAGE); // départ au fond de la zone de départ
	int pp2 = Etape::makeEtape(Position(400, 450, true), Etape::POINT_PASSAGE); // départ au fond de la zone de départ
	int goal = Etape::makeEtape(Position(400, 950, true), Etape::ABEILLE); // départ au fond de la zone de départ
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

bool Coupe2019::updateWIP(goal_strategy::UpdateStrat::Request &req,
           goal_strategy::UpdateStrat::Response &res) {
	std::cout << "updating strat!" << std::endl;
	if (req.obstacle_seen) {
		std::cout << "J'ai vu un Grominet!" << std::endl;
	}
	geometry_msgs::Point point1 = geometry_msgs::Point();
	point1.x = 0;
	point1.y = 0;
	point1.z = 0;
	res.new_goal = point1;
	return true;
}

int main2(int argc, char **argv) {
	//geometry_msgs::Point point1 = geometry_msgs::Point(0, 0, 0);
	geometry_msgs::Point point1 = geometry_msgs::Point();
	point1.x = 0;
	point1.y = 0;
	point1.z = 0;
	std::vector<geometry_msgs::Point> etapes;
	etapes.push_back(point1);
	//std::shared_ptr<Coupe2019> strat_graph = std::make_shared<Coupe2019>(false, etapes);
	//Coupe2019* strat_graph = new Coupe2019(false, etapes);
	std::cout << "Hello world!" << std::endl;
	//ros::init(argc, argv, "goal_strategy_node");
	//ros::NodeHandle node;
	//ros::ServiceServer service = node.advertiseService("update", &Coupe2019::update, strat_graph);
	ROS_INFO("krabi_strat_server is ready");
	ros::spin();
	return 0;
}

