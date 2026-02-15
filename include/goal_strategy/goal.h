#pragma once

#define YEAR_2026

#include <math.h>
#include <memory>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

#include <rclcpp/node.hpp>

#include "builtin_interfaces/msg/duration.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#ifdef YEAR_2025
#include "coupe2025.h"
#else
#include "coupe2026.h"
#endif

#include "goal_strategy/actuators2025.h"
#include "goal_strategy/grabi.h"
#include "krabi_msgs/msg/ax12_info.hpp"
#include "krabi_msgs/msg/servos_cmd.hpp"
#include "krabi_msgs/msg/strat_movement.hpp"

#include "goal_strategy/publisherCreator.h"
#include "goal_strategy/subscriptionCreator.h"
#include "krabilib/pose.h"

// The distance to a goal (in m)
#define REACH_DIST 0.02                            // m
#define REACH_ANG AngleTools::deg2rad(AngleDeg(2)) // °

enum VacuumState
{
    OPEN_AIR,
    WEAK_VACUUM,
    STRONG_VACUUM
};

class GoalStrat : public rclcpp::Node
{
public:
    GoalStrat();
    void loop();
    enum class State
    {
        INIT,
        WAIT_TIRETTE,
        RUN,
        EXIT,
        ALL_DONE
    };
    bool isArrivedAtGoal();
    void stop();
    void initTF();

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
    void setMaxSpeedAtArrival();
    void clamp_mode();
    bool recule(rclcpp::Duration a_time);
    bool recule(rclcpp::Duration a_time, Distance a_distance);
    bool avance(rclcpp::Duration a_time);
    bool avance(rclcpp::Duration a_time, Distance a_distance);

    bool reculeOuAvance(rclcpp::Duration a_time, Distance a_distance, bool sensRecule = true);
    bool reculeDroit(rclcpp::Duration a_time, Distance a_distance);
    bool avanceDroit(rclcpp::Duration a_time, Distance a_distance);
    bool reculeOuAvanceDroit(rclcpp::Duration a_time, Distance a_distance, bool sensRecule = true);
    bool goToDroit(Position& a_position,
                   rclcpp::Duration a_time,
                   Distance a_distance,
                   bool sensRecule,
                   bool a_position_precise);

    void stopAngular();
    void startAngular();
    void stopActuators();
    void stopLinear();
    void startLinear();
    void recalage_bordure();
    void checkStopMatch();
    bool checkFunnyAction();

    void updateCurrentPose();
    void updateRemainingTime(builtin_interfaces::msg::Duration::SharedPtr remainingTime);
    void updateTirette(std_msgs::msg::Bool::SharedPtr tirette);
    void updateVacuum(std_msgs::msg::Float32::SharedPtr vacuum_msg);
    void updateAX12Info(krabi_msgs::msg::AX12Info::SharedPtr ax12_msg, uint8_t);
    void updateAX12Info1(krabi_msgs::msg::AX12Info::SharedPtr ax12_msg)
    {
        updateAX12Info(ax12_msg, 1);
    };
    void updateAX12Info2(krabi_msgs::msg::AX12Info::SharedPtr ax12_msg)
    {
        updateAX12Info(ax12_msg, 2);
    };
    void updateAX12Info3(krabi_msgs::msg::AX12Info::SharedPtr ax12_msg)
    {
        updateAX12Info(ax12_msg, 3);
    };
    void updateAX12Info4(krabi_msgs::msg::AX12Info::SharedPtr ax12_msg)
    {
        updateAX12Info(ax12_msg, 4);
    };
    void updateDigitalReads(std_msgs::msg::Byte::SharedPtr digitalReads);
    void updateOtherRobots(geometry_msgs::msg::PoseArray::SharedPtr);
    void updateStepperElevator(krabi_msgs::msg::InfosStepper::SharedPtr stepper_info_msg)
    {
        m_actuators2025.updateStepperElevator(stepper_info_msg);
    };

    bool isAlignedWithAngle(Angle angle);

    void publishScore();
    void printCurrentAction();
    void publishDebugInfos();
    void publishGoal();
    void publishAll();
    void publishIsBlue();

    void pushBanner();

    void setDistancesFromRobotsToEtapes();

    void publishStratMovement();

    bool isParked();

    bool isArrivedAtIntermediate();
    bool isArrivedAt(Etape* a_etape);
    bool isCurrentEtapeTheGoal();

    State m_state = State::RUN;
    Distance m_dist_to_goal;
    bool m_state_msg_displayed;
#ifdef YEAR_2025
    std::unique_ptr<Coupe2025> m_strat_graph;
    StartingPosition2025 m_starting_position_2025;
#else
    std::unique_ptr<Coupe2026> m_strat_graph;
#endif

    long m_timeout_moving, m_timeout_orient;
    bool m_is_first_action;

    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_is_blue_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_debug_ma_etapes_pub;
    rclcpp::Publisher<krabi_msgs::msg::StratMovement>::SharedPtr m_strat_movement_pub;
    rclcpp::Subscription<builtin_interfaces::msg::Duration>::SharedPtr m_remaining_time_match_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_tirette_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_vacuum_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_other_robots_sub;
    rclcpp::Subscription<krabi_msgs::msg::InfosStepper>::SharedPtr m_elevator_sub;
    rclcpp::Subscription<krabi_msgs::msg::AX12Info>::SharedPtr m_ax12_1_info_sub;
    rclcpp::Subscription<krabi_msgs::msg::AX12Info>::SharedPtr m_ax12_2_info_sub;
    rclcpp::Subscription<krabi_msgs::msg::AX12Info>::SharedPtr m_ax12_3_info_sub;
    rclcpp::Subscription<krabi_msgs::msg::AX12Info>::SharedPtr m_ax12_4_info_sub;
    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr m_digital_reads_sub;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{ nullptr };
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;
    Pose m_current_pose;
    Pose m_goal_pose;

    // ROS Params
    bool m_is_blue; // true if blue
    float goal_MAX_ALLOWED_ANGULAR_SPEED;

    rclcpp::Duration m_remainig_time = rclcpp::Duration(1000, 0);
    Etape::EtapeType m_previous_etape_type;
    bool m_action_aborted;
    float m_score_match;
    float m_score_match_at_end;
    rclcpp::Time m_moving_timeout_deadline;
    bool m_funny_action_counted;
    krabi_msgs::msg::StratMovement m_strat_mvnt;
    bool m_tirette;
    float m_vacuum_level;
    int m_vacuum_state;
    bool m_all_done_do_funny = false;

    Actuators2025 m_actuators2025;
    bool m_goal_init_done;
    bool m_at_least_one_carre_fouille_done;
    std::vector<Position> m_potential_other_robots;

    int m_year;

    std::shared_ptr<Grabi> m_grabi;
    std::shared_ptr<Servomotor> m_servo_banner;

    std::thread m_running;

    int override_gear;
    Etape* m_area_funny = nullptr;
};
