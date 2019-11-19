//main goal.c
#define STANDALONE_STRATEGIE
#define USE_IOSTREAM
#define ENABLE_REAL_SERVOS
#ifdef USE_IOSTREAM
#include <iostream>
#endif
#include <fstream>
#include "Krabi/strategie/etape.h"
#include "Krabi/strategie/dijkstra.h"
//#include "Krabi/goldo2018.h"
#include "goal_strategy/coupe2019.h"
//#include "Krabi/../../stratV3/include/strategie/etape.h"
#include "Krabi/constantes.h"
#include "goal_strategy/goal.h"

#define NB_NEURONS 360

#define SERVO_RIGHT 0
#define SERVO_LEFT  1

#define UP_ANGLE      0.015259
#define DOWN_ANGLE    0.029755
#define RELEASE_ANGLE 0.032043 //2100

#define LOW_SPEED  0.19607 //50
#define FAST_SPEED 0.58823 //150

/*
 * Choose the first attractor
 */
using namespace goal;

static float * set_servo; //array of cmd see order in firmware lib/servos.cpp
static float * read_servo;
static float * ard_servo_running;
static float * ard_servo_reset;
static float * idServo; //set_servo[0]
static float * angleServo; //set_servo[1]
static float * speed;  //set_servo[2]
static float * currentAction; //set_servo[3]
static float * enable;  //set_servo[4]
static float * startFunny;  //set_servo[5]
static float * resetServo;  //set_servo[6]

State state = RUN;

void ard_goToPosition(int servoNb, enum PositionServo position) {
#ifdef ENABLE_REAL_SERVOS
	*idServo = servoNb * 0.0039;
		switch (position) {
			case UP :
				*angleServo = UP_ANGLE;
				*speed = LOW_SPEED;
			break;

			case DOWN :
				*angleServo = DOWN_ANGLE;
				*speed = LOW_SPEED;
			break;

			case RELEASE :
				*angleServo = RELEASE_ANGLE;
				*speed = FAST_SPEED;
			break;

			default :
				;
		}
		printf("Setting servo %d to angle %f with speed %f\n", servoNb, *angleServo, *speed);

		//start servos
		*enable = 1.;
#endif
}

unsigned int get_idx_of_max (float * vector, size_t len) {
	unsigned int curr_max = 0, i;

	for (int i = 0; i < len; i += 1) {
		if (vector[i] > vector[curr_max])
			curr_max = i;
	}

	return curr_max;
}

// Handle CTRL + C
void sig_handler(int signal) {
	static unsigned int nb = 0;
	(void)signal;
	state = EXIT;

	// If this has been called more than once, immediately exit
	if (nb++ >= 1) {
		printf("Immediate exit requested.\n");
		exit(1);
	}
}

void debug_vector (const char * title, const float * vector, const size_t len, const int display_index, const int display_max) {
	unsigned int i, idx;
	float max_value;

	printf("%s",title);

	for (int i = 0; i < len; i += 10) {
		printf("%+.1f ", vector[i]);
	}
	printf("\n");

	if (display_index == TRUE) {
		for (i = 0; i < len; i += 10) {
		       printf("% 4d ", i);
		}

		printf("\n");
	}

	if (display_max == TRUE) {
		idx = 0;
		max_value = vector[0];

		for (int i = 1; i < len; i += 1) {
			if (vector[i] > max_value) {
				idx = i;
				max_value = vector[i];
			}
		}

		printf("Max: %f @%d deg\n", max_value, idx);
	}

	printf("\n");
}

/*
 * Init share memory to store pid
 */
static float * emergency_stop_bis;
void init_ard_shm (void) {
	blc_channel * chan;
	chan  = create_blc_channel( "GOAL_PID" , sizeof(float));
	int * p = (int*)chan->data;
	*p = getpid();

	chan       = create_blc_channel( "MOTORS_CLUB_SpeedOrder" , sizeof(float)*4);
	emergency_stop_bis   = (float*)chan->data;

	chan       =  create_blc_channel( "SERVOS_CLUB_Servos"  , sizeof(float)*7);
	set_servo    = (float*)chan->data;
	ard_servo_reset = &set_servo[6];

	/*
	idServo       = (uint8_t *)&set_servo[0];
	angleServo         = (uint16_t *)&set_servo[1];
	speed         = (uint8_t *)&set_servo[2];
	currentAction = (uint16_t *)&set_servo[3]; //not used for now
	enable        = (uint8_t *)&set_servo[4];
	startFunny    = (uint8_t *)&set_servo[5]; //totaly unused (firmware manage it)
	*/

	idServo       = (float*)&set_servo[0];
	angleServo         = (float *)&set_servo[1];
	speed         = (float *)&set_servo[2];
	currentAction = (float *)&set_servo[3]; //not used for now
	enable        = (float *)&set_servo[4];
	startFunny    = (float *)&set_servo[5]; //totaly unused (firmware manage it)


	//READER
	chan       = create_blc_channel( "SERVOS_CLUB_Servos_status" , sizeof(float)*3);
	read_servo   = (float*)chan->data;

	//set to 1 when broker and arduino is running
	chan       = create_blc_channel( "SERVOS_CLUB_running", sizeof(float));
	ard_servo_running  = (float*)chan->data;
}

void GoalStrat::printCurrentAction() {
	if (!state_msg_displayed) {
		int etapeId = strat_graph->getEtapeEnCours()->getNumero();
		Position goal = strat_graph->getEtapeEnCours()->getPosition();
		int mission_type = strat_graph->getEtapeEnCours()->getEtapeType();
		std::cout << "Attractor: " << etapeId << std::endl;
		std::cout << "goal: " << goal.Print() << std::endl;
		std::cout << "type: " << mission_type << std::endl;
		state_msg_displayed = true;
	}
	return;
	fflush(stdout);
}



// Entry point
int main (int argc, char * argv[]) {
	GoalStrat* goalStrat = new GoalStrat{};

	while(42) {
		goalStrat->loop();
	}
}

void GoalStrat::orient_to_angle(float a_angle) {
        orientation = strategy.input->orientation;

	for (int i = 0; i < NB_NEURONS; i += 1) {
		strategy.output->neural_field[i] = cos((int) (i - a_angle) * M_PI / 180.);
	}
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
	// Go fetch the food
	// Orient the robot
	move_toward_goal();

	int driveAngle = get_idx_of_max(attractor.drive, NB_NEURONS);

	// Get distance to the attractor
	dist_to_goal = attractor.drive[driveAngle];

	//printf("Distance to objective: %.2f\n", dist_to_goal);
	//fflush(stdout);

	if (dist_to_goal < REACH_DIST) {
		// Make it stop
		strategy.output->speed_inhibition = 0;

		return 1;
	}
	return 0;
}

int GoalStrat::done_orienting_to(int angle) {
        // Output a goal relative to the robot
	orient_to_angle(angle);

	// Compute angular diff
	unsigned int angular_error = compute_angular_diff(angle, orientation);

	// When we reached the correct orientation, angularly stop and switch state
	if (angular_error < REACH_ANG) {
		return 1;
	}
	return 0;
}

void GoalStrat::move_toward_goal() {

	attractor = strategy.input->attractors[goal_nb];
	memcpy(strategy.output->neural_field, attractor.drive, NB_NEURONS * sizeof(*(attractor.drive)));

	unsigned int angular_diff = get_angular_diff();

	strategy.output->speed_inhibition = (int)round(MAX_ALLOWED_SPEED * (- (float)angular_diff / 180. + 1.));
}

unsigned int GoalStrat::get_angular_diff() {
	attractor = strategy.input->attractors[goal_nb];
	// Get distance to the attractor
	int driveAngle = get_idx_of_max(attractor.drive, NB_NEURONS);

	// Inhibit linear speed based on angular difference
	// Compute the angular distance between our orientation and the target's
	unsigned int ad1 = abs(360 + (int) driveAngle - (int)strategy.input->orientation) % 360;
	unsigned int ad2 = abs(360 + (int)strategy.input->orientation - (int) driveAngle) % 360;

	if (ad1 < ad2) {
		return ad1;
	}
	return ad2;
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

void GoalStrat::go_to_next_mission() {
	//strategy.output->angular_speed_inhibition = 0;
	isFirstAction = false;
	// Reset timeout
	clock_gettime (CLOCK_MONOTONIC, &begin);
	
	strategy.output->angular_speed_inhibition = MAX_ALLOWED_ANGULAR_SPEED;
	state_msg_displayed = false;

	int strat_graph_status = strat_graph->update();
	if (strat_graph_status == -1) {
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
	// Attach signal handler
	signal(SIGINT, sig_handler);
	init_ard_shm();

	/*************************************************
	 *           Variables initialization            *
	 *************************************************/

	// Create & attach to SHM segment
	strategy = init_strategy(SHM_INPUT_FILE, GOAL_SHM_FILE, AM_STRATEGY);
	mission_finished = 0;
	mission_state = GO_FOOD_1;

	/*************************************************
	 *                   Main loop                   *
	 *************************************************/

	printf("Starting goal strategy.\n");
	attractor = strategy.input->attractors[goal_nb];
	int driveAngle = get_idx_of_max(attractor.drive, NB_NEURONS);
	printf("Start goal #%d @ %d deg.\n", goal_nb, driveAngle);

	strategy.output->angular_speed_inhibition = MAX_ALLOWED_ANGULAR_SPEED;


	printf("before\n");
	Position pos(200,1850, true);//strategy.input->color);//1500, isBlue());
	printf("after\n");
	startingPosition = PositionPlusAngle (pos,-M_PI/2);

	attractor = strategy.input->attractors[goal_nb];

	ard_goToPosition(SERVO_RIGHT, UP);
	ard_goToPosition(SERVO_LEFT, UP);
	usleep(1000000);
	ard_goToPosition(SERVO_RIGHT, DOWN);
	ard_goToPosition(SERVO_LEFT, DOWN);
	usleep(1000000);
	ard_goToPosition(SERVO_RIGHT, UP);
	ard_goToPosition(SERVO_LEFT, UP);

	//std::cout << "size of attractors = " << strategy.input->attractors.size() << std::endl;
	strat_graph = new Coupe2019(false, strategy.input->attractors, 8);
	//while(sendNewMission(strat_graph) != -1) {}
	strat_graph->update();
	
	fflush(stdout);

	// Initialize time
	clock_gettime (CLOCK_MONOTONIC, &begin);
	timeoutMoving = 10;// sec
	timeoutOrient = 5;// sec
	isFirstAction = true;
}


/**
 * Update the strategy, and send the new mission
 * @param StrategieV3* strat, the strategy
 * @return result, the strategie's update result
 */
int GoalStrat::sendNewMission(StrategieV3* strat) {
	int result = strat->update();
	int etapeId = strat->getEtapeEnCours()->getNumero();
	Position goal = strat->getEtapeEnCours()->getPosition();
	int mission_type = strat->getEtapeEnCours()->getEtapeType();
    	#ifdef USE_IOSTREAM
	std::cout << "Attractor: " << etapeId << std::endl;
	std::cout << "goal: " << goal.Print() << std::endl;
	std::cout << "type: " << mission_type << std::endl;
	#endif // USE_IOSTREAM
    	PositionPlusAngle* goalWithAngle = new PositionPlusAngle(goal, 0);
	return result;
}

int GoalStrat::loop() {	
	while (state != EXIT) {
		int driveAngle;
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
					strategy.output->speed_inhibition = 0;

					printf("In front of goldenium/accelerator, orienting\n");
					fflush(stdout);
					angleAction = 320;
					if (strategy.input->color) {
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
					if (strategy.input->color) {
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

		strategy.output->strength = 1;
		usleep(20000);
	}
	std::cout << "Mission accomplished, shutting down" << std::endl;	
	fflush(stdout);
	// Stop everything
	strategy.output->speed_inhibition = 0;
	strategy.output->angular_speed_inhibition = 0;


	// When exiting, set strength to 1: we want the robot to stay off

	strategy.output->strength = 1;

	// Cleanly close strategy
	close_strategy(&strategy);

	return 0;
}

