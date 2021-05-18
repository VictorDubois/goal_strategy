#pragma once

#include <math.h>
#include <memory>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Duration.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include "coupe2021.h"
#include "goal_strategy/actuators.h"
#include "goal_strategy/grabber.h"
#include "krabi_msgs/servos_cmd.h"
#include "krabi_msgs/strat_movement.h"
#include "krabilib/pose.h"

// The distance to a goal (in m)
#define REACH_DIST 0.02                            // m
#define REACH_ANG AngleTools::deg2rad(AngleDeg(2)) //°

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
    enum class State
    {
        INIT,
        WAIT_TIRETTE,
        RUN,
        EXIT
    };

private:
    void init();
    void stateRun();
    void stateExit();

    int chooseAngle();
    void chooseGear();
    void abortAction();
    void alignWithAngle(Angle a_angle);
    bool alignWithAngleWithTimeout(Angle angle);
    void goToNextMission();
    void hissezLesPavillons();
    void moveArm(enum PositionServo position);
    void setMaxSpeedAtArrival();
    void stopActuators();
    void stopLinear();
    void startLinear();
    void checkStopMatch();
    void checkFunnyAction();

    void updateCurrentPose();
    void updateRemainingTime(std_msgs::Duration remainingTime);
    void updateTirette(std_msgs::Bool tirette);

    bool isAlignedWithAngle(Angle angle);
    bool isArrivedAtGoal();

    void publishScore();
    void printCurrentAction();
    void publishDebugInfos();
    void publishGoal();

    State m_state = State::RUN;
    Distance m_dist_to_goal;
    bool m_state_msg_displayed;
    std::unique_ptr<Coupe2021> m_strat_graph;
    long m_timeout_moving, m_timeout_orient;
    bool m_is_first_action;

    ros::NodeHandle m_nh;

    ros::Publisher m_goal_pose_pub;
    ros::Publisher m_arm_servo_pub;
    ros::Publisher m_stop_linear_pub;
    ros::Publisher m_score_pub;
    ros::Publisher m_debug_ma_etapes_pub;
    ros::Publisher m_strat_movement_pub;

    ros::Subscriber m_remaining_time_match_sub;
    ros::Subscriber m_tirette_sub;

    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener;
    Pose m_current_pose;
    Pose m_goal_pose;

    krabi_msgs::servos_cmd m_servos_cmd;
    ros::Duration m_remainig_time;
    bool m_is_blue; // true if blue
    Etape::EtapeType m_previous_etape_type;
    bool m_action_aborted;
    float m_score_match;
    bool m_first_manche_a_air_done;
    bool m_servo_out;
    ros::Time m_moving_timeout_deadline;
    bool m_funny_action_counted;
    krabi_msgs::strat_movement m_strat_mvnt;
    bool m_tirette;
    Actuators m_actuators;

    Grabber theThing;
};
