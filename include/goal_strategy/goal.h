#pragma once

#include <math.h>
#include <memory>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

#include <geometry_msgs/msg/pose_array.hpp>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
//#include <builtin_interfaces/msg/duration.hpp>
#include "builtin_interfaces/msg/duration.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "coupe2023.h"
#include "goal_strategy/actuators.h"
#include "goal_strategy/actuators2023.h"
#include "goal_strategy/claws.h"
#include "goal_strategy/grabber.h"
#include "krabi_msgs/msg/servos_cmd.hpp"
#include "krabi_msgs/msg/strat_movement.hpp"
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

class GoalStrat: public rclcpp::Node
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
    void recule(rclcpp::Duration a_time);
    void recule(rclcpp::Duration a_time, Distance a_distance);
    void reculeDroit(rclcpp::Duration a_time, Distance a_distance);
    void stopAngular();
    void startAngular();
    void stopActuators();
    void stopLinear();
    void startLinear();
    void recalage_bordure();
    void checkStopMatch();
    bool checkFunnyAction();

    void updateCurrentPose();
    void updateRemainingTime(builtin_interfaces::msg::Duration remainingTime);
    void updateTirette(std_msgs::msg::Bool tirette);
    void updateVacuum(std_msgs::msg::Float32 vacuum_msg);
    void updateOtherRobots(geometry_msgs::msg::PoseArray);

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

    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose_pub;
    rclcpp::Publisher<krabi_msgs::msg::ServosCmd>::SharedPtr m_arm_servo_pub;
    //rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_stop_linear_pub;
    //rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_score_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_debug_ma_etapes_pub;
    rclcpp::Publisher<krabi_msgs::msg::StratMovement>::SharedPtr m_strat_movement_pub;

    rclcpp::Subscription<builtin_interfaces::msg::Duration>::SharedPtr m_remaining_time_match_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_tirette_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_vacuum_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_other_robots_sub;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;
    Pose m_current_pose;
    Pose m_goal_pose;

    krabi_msgs::msg::ServosCmd m_servos_cmd;
    rclcpp::Duration m_remainig_time = rclcpp::Duration(1000,0);
    bool m_is_blue; // true if blue
    Etape::EtapeType m_previous_etape_type;
    bool m_action_aborted;
    float m_score_match;
    float m_score_match_at_end;
    bool m_first_manche_a_air_done;
    bool m_servo_out;
    rclcpp::Time m_moving_timeout_deadline;
    bool m_funny_action_counted;
    krabi_msgs::msg::StratMovement m_strat_mvnt;
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
