#ifndef _GOAL_H_
#define _GOAL_H_

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <csignal>
#include <cstring>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <ctime>
#include "blc_channel.h"
#include "attractors.h"
#include "Krabi/strategie/strategies.h"
#include "Krabi/strategie/positionPlusAngle.h"
//#include "goldo2018.h"
#include "coupe2019.h"
#define TRUE 1
#define FALSE 0

#define GOAL_SHM_FILE "goal.key"

// The distance to a goal (in m)
#define REACH_DIST 0.07 // 5cm
#define REACH_ANG 2 //°
enum MissionState {
	GO_FOOD_1,
	PICKUP_FOOD_1,
	GO_DELIVERY_PLACE_1,
	DELIVER_FOOD_1,
	GO_FOOD_2,
	PICKUP_FOOD_2,
	GO_DELIVERY_PLACE_2,
	DELIVER_FOOD_2,
	FOOD_DONE
};

namespace goal {
enum State {
	RUN,
	EXIT
};
}
enum PositionServo {
	UP,
	DOWN,
	RELEASE
};

class GoalStrat
{
public:
	GoalStrat(); 
	int loop();
	void printCurrentAction();
	int done_orienting_to(int angle);
	int arrived_there();
private:
	void orient_to_angle(float a_angle);
	int choose_angle();
	float compute_angular_diff(float, float);
	void move_toward_goal();
	void go_to_next_mission();
	void update_selected_attractor();
	unsigned int get_angular_diff();
	int sendNewMission(StrategieV3* strat);
	bool displayed_end_msg;
	struct Strategy strategy;
	struct Attractor attractor;
	enum MissionState mission_state;
	int goal_nb, orientation, mission_finished;
	float dist_to_goal;
	bool state_msg_displayed;
	PositionPlusAngle startingPosition;
	StrategieV3* strat_graph;
	struct timespec now, begin, orientTime;
	unsigned long timeoutMoving, timeoutOrient;
	bool isFirstAction;
	bool is_baffe_action();
	std::string read_stop_distance_modulation();
	void write_stop_distance_modulation(std::string distanceToWrite);
};

#endif
