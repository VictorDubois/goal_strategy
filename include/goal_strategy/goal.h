#pragma once

#include <math.h>
#include <memory>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include "coupe2023.h"
#include "goal_strategy/actuators.h"
#include "goal_strategy/actuators2023.h"
#include "goal_strategy/claws.h"
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

enum VacuumState
{
    OPEN_AIR,
    WEAK_VACUUM,
    STRONG_VACUUM
};

class GoalStrat
{
public:
    GoalStrat();
    void loop();
    enum class State
    {
        INIT,
        WAIT_TIRETTE,
        RUN,
        EXIT
    };
    bool isArrivedAtGoal();
    void stop();

private:
    void init();
    void stateRun();
    void stateExit();

    int chooseAngle();
    void chooseGear();
    void chooseEffector(bool enable = true);
    void abortAction();
    void alignWithAngle(Angle a_angle);
    bool alignWithAngleWithTimeout(Angle angle);
    void goToNextMission();
    void hissezLesPavillons();
    void moveArm(enum PositionServo position);
    void setMaxSpeedAtArrival();
    void clamp_mode();
    void recule(ros::Duration a_time);
    void recule(ros::Duration a_time, Distance a_distance);
    void reculeDroit(ros::Duration a_time, Distance a_distance);
    void stopAngular();
    void startAngular();
    void stopActuators();
    void stopLinear();
    void startLinear();
    void recalage_bordure();
    void checkStopMatch();
    bool checkFunnyAction();

    void updateCurrentPose();
    void updateRemainingTime(std_msgs::Duration remainingTime);
    void updateTirette(std_msgs::Bool tirette);
    void updateVacuum(std_msgs::Float32 vacuum_msg);
    void updateOtherRobots(geometry_msgs::PoseArray);

    bool isAlignedWithAngle(Angle angle);
    bool isArrivedAtGoal(Distance a_offset);

    void publishScore();
    void printCurrentAction();
    void publishDebugInfos();
    void publishGoal();
    void publishAll();

    void pushCarreFouille();
    void retractePusher();
    void dropCherries();
    void closeCherriesDispenser();

    void setDistancesFromRobotsToEtapes();

    void publishStratMovement();

    State m_state = State::RUN;
    Distance m_dist_to_goal;
    bool m_state_msg_displayed;
    std::unique_ptr<Coupe2023> m_strat_graph;
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
    ros::Subscriber m_vacuum_sub;
    ros::Subscriber m_weathercock_state_sub;
    ros::Subscriber m_other_robots_sub;

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
    float m_score_match_at_end;
    bool m_first_manche_a_air_done;
    bool m_servo_out;
    ros::Time m_moving_timeout_deadline;
    bool m_funny_action_counted;
    krabi_msgs::strat_movement m_strat_mvnt;
    bool m_tirette;
    float m_vacuum_level;
    int m_vacuum_state;
    Actuators m_actuators;
    Actuators2023 m_actuators2023;
    bool m_goal_init_done;
    bool m_at_least_one_carre_fouille_done;
    std::vector<Position> m_potential_other_robots;

    int m_year;

    std::shared_ptr<Grabber> m_theThing;
    std::shared_ptr<Claws> m_claws;
    std::shared_ptr<Servomotor> m_servo_pusher;
    std::shared_ptr<Servomotor> m_servo_cherries;

    std::thread m_running;

    int override_gear;
    Assiette* m_assiette_funny = nullptr;

    bool m_claws_openned_once = false;
};
