#pragma once

#include <csignal>
#include <cstring>
#include <ctime>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <unistd.h>
//#include "goldo2018.h"
#include "coupe2019.h"
#include  "krabi_msgs/servos_cmd.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Duration.h"


#define GOAL_SHM_FILE "goal.key"

// The distance to a goal (in m)
#define REACH_DIST 0.05//0.07 // 5cm
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
enum PositionServo
{
    UP,
    DOWN,
    RELEASE,
    OUT,
    IN,
    FOLDED
};

class GoalStrat
{
public:
    GoalStrat();
    int loop();
    void printCurrentAction();
    int done_orienting_to(Angle angle);
    int arrived_there();

private:
    void orient_to_angle(Angle a_angle);
    int choose_angle();
    void move_toward_goal();
    void go_to_next_mission();
    unsigned int get_angular_diff();
    void updateCurrentPose(geometry_msgs::Pose);
    void moveArm(enum PositionServo position);
    bool is_baffe_action();
    std::string read_stop_distance_modulation();
    void write_stop_distance_modulation(std::string distanceToWrite);
    void updateRemainingTime(std_msgs::Duration remainingTime);
    void hissezLesPavillons();
    void checkFunnyAction();
    void orient_to_angle_with_timeout(Angle angleIfBlue, Angle angleIfNotBlue);
    bool isBlue();
    void publishEtapes();
    void chooseGear();
    void abortAction();
    void publishScore();
    void updateGirouette(std_msgs::Bool girouette_is_south);
    void stopActuators();
    void stopLinear();
    void startLinear();
    void checkStopMatch();

    bool displayed_end_msg;
    int orientation, mission_finished;
    float dist_to_goal;
    bool state_msg_displayed;
    Pose startingPosition;
    Coupe2019* strat_graph;
    struct timespec now, begin, orientTime;
    long timeoutMoving, timeoutOrient;
    bool isFirstAction;
    ros::Publisher goal_pose_pub;
    ros::Publisher arm_servo_pub;
    ros::Publisher goals_pub;
    ros::Publisher reverse_pub;
    ros::Publisher stop_linear_pub;
    ros::Publisher score_pub;
    ros::Subscriber current_pose_sub;
    ros::Subscriber remaining_time_match_sub;
    ros::Subscriber color_sub;
    ros::Subscriber girouette_sub;
    Pose currentPosition;
    Pose goal_pose;
    krabi_msgs::servos_cmd m_servos_cmd;
    ros::Duration remainig_time;
    bool team_color; // true if blue
    Etape::EtapeType previousEtapeType;
    bool actionaborted;
    float scoreMatch;
    Etape::EtapeType m_good_mouillage;
    bool firstMancheAAirdone;
    bool servo_out;
    ros::Time movingTimeoutDeadline;
    bool funny_action_counted;
};
