#ifndef COUPE2019_H
#define COUPE2019_H

#define ETAPE_GARAGE 1

#define NOMBRE_ETAPES 30

#include "Krabi/strategie/strategiev3.h"
//#include "Krabi/strategie/attractors.h"
#include "Krabi/strategie/etape.h"
#include "Krabi/strategie/abeille.h"
#include "Krabi/strategie/goldenium.h"
#include "Krabi/strategie/accelerator.h"
#include "Krabi/strategie/reservoirEau.h"
#include <geometry_msgs/Point.h>

#include <geometry_msgs/Pose.h>
#include "goal_strategy/UpdateStrat.h"

class Coupe2019 : public StrategieV3
{
public:
	/** @brief Constructeur de la stratégie *
	* @param isBlue le côté de départ */
	//Coupe2019(bool isYellow, Attractor* attractors, unsigned int sizeofAttractors);
	Coupe2019(bool isYellow, std::vector<geometry_msgs::Pose> etapes);
	Coupe2019(bool isYellow, std::vector<geometry_msgs::Point> etapes);
	//Position positionFromAttractor(int attractorId);
bool updateWIP(goal_strategy::UpdateStrat::Request &req,
           goal_strategy::UpdateStrat::Response &res);

	/** @brief convert ROS's Point to position */
	Position positionFromPoint(geometry_msgs::Point point);
	geometry_msgs::Point pointFromPosition(Position position);

	/** @brief convert ROS's Pose to position */
	Position positionFromPose(geometry_msgs::Pose position);
	geometry_msgs::Pose poseFromPosition(Position position);
private:

//	/** Nombre d'étapes dans le graph */
//	int nombreEtapes = NOMBRE_ETAPES;

//	/** Numéro de l'étape où le robot va se cacher à la fin */
//	int numeroEtapeGarage = ETAPE_GARAGE;

	/** @brief update du score d'une étape */
	int getScoreEtape(int i);

	//Attractor* m_attractors;

};


#endif // COUPE2019_H
