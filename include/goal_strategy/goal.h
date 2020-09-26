#ifndef _GOAL_H_
#define _GOAL_H_

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
#include "goal_strategy/servos_cmd.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Duration.h"
#define TRUE 1
#define FALSE 0

#define GOAL_SHM_FILE "goal.key"

// The distance to a goal (in m)
#define REACH_DIST 20 // 0.07 // 20mm
#define REACH_ANG 2   //°
namespace goal
{
enum State
{
    RUN,
    EXIT
};
}
enum PositionServo
{
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
    int done_orienting_to(float angle);
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
    void updateCurrentPose(geometry_msgs::Pose);
    void moveArm(enum PositionServo position);
    bool is_baffe_action();
    std::string read_stop_distance_modulation();
    void write_stop_distance_modulation(std::string distanceToWrite);
    void updateRemainingTime(std_msgs::Duration remainingTime);
    void updateColor(std_msgs::Bool is_blue);
    void hissezLesPavillons();
    void checkFunnyAction();
    void orient_to_angle_with_timeout(float angleIfBlue, float angleIfNotBlue);
    bool isBlue();

    bool displayed_end_msg;
    int goal_nb, orientation, mission_finished;
    float dist_to_goal;
    bool state_msg_displayed;
    PositionPlusAngle startingPosition;
    StrategieV3* strat_graph;
    struct timespec now, begin, orientTime;
    long timeoutMoving, timeoutOrient;
    bool isFirstAction;
    ros::Publisher goal_pose_pub;
    ros::Publisher arm_servo_pub;
    ros::Subscriber current_pose_sub;
    ros::Subscriber remaining_time_match_sub;
    ros::Subscriber color_sub;
    PositionPlusAngle currentPosition;
    PositionPlusAngle goal_pose;
    goal_strategy::servos_cmd m_servos_cmd;
    ros::Duration remainig_time;
    bool team_color; // true if blue
};

#endif
