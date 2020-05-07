#include <iostream>
#include <fstream>
#include "Krabi/strategie/etape.h"
#include "Krabi/strategie/dijkstra.h"
#include "Krabi/positionPlusAngle.h"
//#include "Krabi/goldo2018.h"
#include "goal_strategy/coupe2019.h"
//#include "Krabi/../../stratV3/include/strategie/etape.h"
#include "Krabi/constantes.h"
#include "goal_strategy/goal.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_datatypes.h"

#define SERVO_RIGHT 0
#define SERVO_LEFT  1

#define UP_ANGLE      0.015259
#define DOWN_ANGLE    0.029755
#define RELEASE_ANGLE 0.032043 //2100

#define LOW_SPEED  0.19607 //50
#define FAST_SPEED 0.58823 //150

using namespace goal;

State state = RUN;

void ard_goToPosition(int servoNb, enum PositionServo position) {
}

void stopLinear() {
	//@TODO stop linear speed
}

void startLinear() {
	//@TODO stop linear speed
}

bool isBlue() {
	//@TODO return true if current color is Blue
	return true;
}

void GoalStrat::printCurrentAction() {
	if (!state_msg_displayed) {
		int etapeId = strat_graph->getEtapeEnCours()->getNumero();
		Position goal = strat_graph->getEtapeEnCours()->getPosition();
		int mission_type = strat_graph->getEtapeEnCours()->getEtapeType();
		std::cout << "Etape id: " << etapeId << std::endl;
		std::cout << "goal: " << goal.Print() << std::endl;
		std::cout << "type: " << mission_type << std::endl;
		state_msg_displayed = true;
	}
	return;
	fflush(stdout);
}

// Entry point
int main (int argc, char * argv[]) {
	ros::init(argc, argv, "goalStrat");
	GoalStrat* goalStrat = new GoalStrat{};

	goalStrat->loop();
}

void GoalStrat::orient_to_angle(float a_angle) {
	goal_pose.setAngle(a_angle);
}

float GoalStrat::compute_angular_diff(float a_angle_1, float a_angle_2) {
	if (a_angle_1 >= a_angle_2) {
		return a_angle_1 - a_angle_2;
	}
	return a_angle_2 - a_angle_1;
}

void GoalStrat::update_selected_attractor() {
	// Choose correct attractor number
	goal_nb = strat_graph->getEtapeEnCours()->getNumero() - 1;// -1 because the start is not an attractor
	return;
}

int GoalStrat::arrived_there() {

	dist_to_goal = (currentPosition.getPosition()-strat_graph->getEtapeEnCours()->getPosition()).getNorme();
    std::cout << "current pose: x = " << currentPosition.getPosition().getX() << ", y = " << currentPosition.getPosition().getY() << ", etape_en_cours: x = " << strat_graph->getEtapeEnCours()->getPosition().getX() << ", y = " << strat_graph->getEtapeEnCours()->getPosition().getY()<< std::endl;
    printf("Distance to objective: %.2f\n", dist_to_goal);
    fflush(stdout);

	if (dist_to_goal < REACH_DIST) {
		// Make it stop
		stopLinear();

		return 1;
	}
	return 0;
}

int GoalStrat::done_orienting_to(int angle) {
        // Output a goal relative to the robot
	orient_to_angle(angle);

	// Compute angular diff
	unsigned int angular_error = compute_angular_diff(angle, currentPosition.getAngle());

	// When we reached the correct orientation, angularly stop and switch state
	if (angular_error < REACH_ANG) {
		return 1;
	}
	return 0;
}

void GoalStrat::move_toward_goal() {
	Position goal_position = strat_graph->getEtapeEnCours()->getPosition();
	goal_pose.setX(goal_position.getX());
	goal_pose.setY(goal_position.getY());
	goal_pose_pub.publish(goal_pose.getPose());
}

unsigned int GoalStrat::get_angular_diff() {
	//@TODO return the angular difference between the command and the current state
	return 0;
}

// Returns true if the current action is a baffe (push goldenium/accelerator). Specific to 2019
bool GoalStrat::is_baffe_action() {
	return strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::ACCELERATOR
	    || strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::GOLDENIUM ;
}

std::string GoalStrat::read_stop_distance_modulation() {
	try {
		std::ifstream stopDistanceFile;
		std::string stopDistanceRead = "";
		stopDistanceFile.open ("/tmp/stopdistance");
		if (stopDistanceFile.is_open()) {
			std::getline(stopDistanceFile, stopDistanceRead);
			stopDistanceFile.close();
			return stopDistanceRead;
		}
		else {
			return "";
		}
	}
	catch (const std::exception& e) {
		std::cout << "Error caught while trying to get distanceCoeff in goal! " << e.what() << std::endl;
		fflush(stdout);
		return "";
	}
}

// Writes the modulation of detection distance. 1 = full distance, 0.5 = start braking when the obstacle is closer
void GoalStrat::write_stop_distance_modulation(std::string distanceToWrite) {
	if (read_stop_distance_modulation() == distanceToWrite)
	{
		// No need to write it
		return;
	}

	try {
		std::ofstream writeStopFile;
		writeStopFile.open ("/tmp/stopdistance", std::ios::trunc);
		if (writeStopFile.is_open()) {
			writeStopFile << distanceToWrite << std::endl;
			writeStopFile.close();
		}
	}
        catch (const std::exception& e) {
                std::cout << "Error caught while trying to write distanceCoeff! " << e.what() << std::endl;
		fflush(stdout);
        }
}

void GoalStrat::updateCurrentPose(geometry_msgs::Pose newPose) {
	currentPosition = PositionPlusAngle(newPose);
}

void GoalStrat::go_to_next_mission() {
	startLinear();
	isFirstAction = false;
	// Reset timeout
	clock_gettime (CLOCK_MONOTONIC, &begin);
	
	state_msg_displayed = false;

	int strat_graph_status = strat_graph->update();
	if (strat_graph_status == -1) {
		std::cout << "Graph status is -1: we're done" << std::endl;
		state = EXIT;
		return;
	}

	if (is_baffe_action()) {
		write_stop_distance_modulation("0.67");
	}
	else {
		write_stop_distance_modulation("1");
	}
	
	//mission_state = strat_graph->getEtapeEnCours()->getEtapeType();
	return;
}

GoalStrat::GoalStrat() {
	// Initialize stop distance modulation
	write_stop_distance_modulation("1");

	usleep(1000000);
	displayed_end_msg = false;
	goal_nb = 0;
	dist_to_goal = 100.;
	state_msg_displayed = false;

	/*************************************************
	 *           Variables initialization            *
	 *************************************************/

	// Create & attach to SHM segment
	mission_finished = 0;
	mission_state = GO_FOOD_1;

	/*************************************************
	 *                   Main loop                   *
	 *************************************************/

	Position pos(200,1850, true);//strategy.input->color);//1500, isBlue());
	startingPosition = PositionPlusAngle (pos,-M_PI/2);

	ard_goToPosition(SERVO_RIGHT, UP);
	ard_goToPosition(SERVO_LEFT, UP);
	usleep(1000000);
	ard_goToPosition(SERVO_RIGHT, DOWN);
	ard_goToPosition(SERVO_LEFT, DOWN);
	usleep(1000000);
	ard_goToPosition(SERVO_RIGHT, UP);
	ard_goToPosition(SERVO_LEFT, UP);

	geometry_msgs::Point point1 = geometry_msgs::Point();
	point1.x = 0;
	point1.y = 0;
	point1.z = 0;
	std::vector<geometry_msgs::Point> etapes;
	etapes.push_back(point1);

	strat_graph = new Coupe2019(false, etapes);
	//while(sendNewMission(strat_graph) != -1) {}
	strat_graph->update();
	
	fflush(stdout);

	// Initialize time
	clock_gettime (CLOCK_MONOTONIC, &begin);
	timeoutMoving = 100;// sec
	timeoutOrient = 50;// sec
	isFirstAction = true;
	ros::NodeHandle n;
    goal_pose_pub = n.advertise<geometry_msgs::Pose>("goal_pose", 1000);
	current_pose_sub = n.subscribe("current_pose", 1000, &GoalStrat::updateCurrentPose, this);
}


/**
 * Update the strategy, and send the new mission
 * @param StrategieV3* strat, the strategy
 * @return result, the strategie's update result
 */
int GoalStrat::sendNewMission(StrategieV3* strat) {
	int result = strat->update();
	int etapeId = strat->getEtapeEnCours()->getNumero();
	//Position goal = strat->getEtapeEnCours()->getPosition();
	int mission_type = strat->getEtapeEnCours()->getEtapeType();
    	#ifdef USE_IOSTREAM
	std::cout << "EtapeId: " << etapeId << std::endl;
	//std::cout << "goal: " << goal.Print() << std::endl;
	std::cout << "type: " << mission_type << std::endl;
	#endif // USE_IOSTREAM
    	//PositionPlusAngle* goalWithAngle = new PositionPlusAngle(goal, 0);
	return result;
}

int GoalStrat::loop() {	
	while (state != EXIT && ros::ok()) {
		bool isLate = false;
		printCurrentAction();
		update_selected_attractor();

		clock_gettime (CLOCK_MONOTONIC, &now);
		if (!isFirstAction && ((now.tv_sec - begin.tv_sec) >= timeoutMoving)) {
			isLate = true;
			std::cout << "Robot is late (spent more than " << timeoutMoving << "seconds trying to reach destination)" << std::endl;	
			fflush(stdout);
		}
			
		if (isLate && !is_baffe_action()) {
			// If it has been too long
			// Then it means that there is an obstacle on the way
			// The first action is excluded because we wait for the tirette (+ no reason for an opponent + no other way)
			// The baffe actions are excluded because if something is detected, it must be the wall
			std::cout << "Timeout, probable obstacle on the way. Trying another path." << std::endl;	
			fflush(stdout);
			strat_graph->collisionAvoided();
			go_to_next_mission();
		}
		else if (arrived_there() || (isLate && is_baffe_action())) {
			//if (angle != -1 || done_orienting_to(angle)) {
			int angleAction = 0;
			switch(strat_graph->getEtapeEnCours()->getEtapeType()) {
				case Etape::EtapeType::ACCELERATOR:
				case Etape::EtapeType::GOLDENIUM:
					// Stop moving, not already done if baffing
					stopLinear();

					printf("In front of goldenium/accelerator, orienting\n");
					fflush(stdout);
					angleAction = 320;
					if (isBlue()) {
						angleAction = 40;
					}
					clock_gettime (CLOCK_MONOTONIC, &orientTime);
					clock_gettime (CLOCK_MONOTONIC, &now);
					while(!done_orienting_to(angleAction) && ((now.tv_sec - orientTime.tv_sec) < timeoutOrient)) {
						clock_gettime (CLOCK_MONOTONIC, &now);
						usleep(10000);
					}
					printf("MOVING SERVO DOWN\n");
					fflush(stdout);
					// Move (both) servos to low position
					ard_goToPosition(SERVO_RIGHT, DOWN);
					ard_goToPosition(SERVO_LEFT, DOWN);
					usleep(1500000); // 1.5s
					angleAction = 220;
					if (isBlue()) {
						angleAction = 140;
					}
					clock_gettime (CLOCK_MONOTONIC, &orientTime);
					clock_gettime (CLOCK_MONOTONIC, &now);
					while(!done_orienting_to(angleAction) && ((now.tv_sec - orientTime.tv_sec) < timeoutOrient)) {
						clock_gettime (CLOCK_MONOTONIC, &now);
						usleep(10000);
					}

					printf("MOVING SERVO UP\n");
					fflush(stdout);
					// Move (both) servos to up position
					ard_goToPosition(SERVO_RIGHT, UP);
					ard_goToPosition(SERVO_LEFT, UP);
					usleep(1500000); // 1.5s
					break;
				default:
					printf("No special action here\n");
					fflush(stdout);
					break;

			}
			go_to_next_mission();
		}
		move_toward_goal();

		usleep(20000);
		
		ros::spinOnce();
	}
	std::cout << "Mission accomplished, shutting down" << std::endl;	
	fflush(stdout);
	return 0;
}

