#pragma once

#define YEAR_2026

#include <atomic>
#include <math.h>
#include <memory>
#include <mutex>
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
#include <std_msgs/msg/string.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <diagnostic_updater/diagnostic_updater.hpp>

#ifdef YEAR_2025
#include "coupe2025.h"
#include "goal_strategy/grabi.h"
#elif defined(YEAR_2026)
#include "coupe2026.h"
#include "goal_strategy/billig.h"
#endif
#include "goal_strategy/actuators2025.h"

#include "krabi_msgs/msg/ax12_info.hpp"
#include "krabi_msgs/msg/caisses_sides.hpp"
#include "krabi_msgs/msg/recalage_zones.hpp"
#include "krabi_msgs/msg/servos_cmd.hpp"
#include "krabi_msgs/msg/strat_movement.hpp"
#include "krabi_msgs/srv/grab_flip_drop.hpp"
#include "krabi_msgs/msg/motors_current.hpp"
#include "krabi_msgs/srv/correct_pose.hpp"
#include "krabi_msgs/srv/start_recalage.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "goal_strategy/publisherCreator.h"
#include "goal_strategy/recalage_sequence.h"
#include "goal_strategy/subscriptionCreator.h"
#include "krabilib/pose.h"
#include "krabilib/tirette_filter.h"

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
    ~GoalStrat();
    void loop();
    enum class State
    {
        INIT,
        WAIT_TIRETTE_IN,
        WAIT_TIRETTE,
        RUN,
        EXIT,
        ALL_DONE
    };
    bool isArrivedAtGoal();
    void stop();
    void initTF();

    // Read-only accessor for tests (e.g. checking the tirette -> RUN transition).
    State getState() const
    {
        return m_state;
    }
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

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

    // ---- Recalage automatique sur bordure ----
    bool recalageBordure(Wall a_wall, double a_clearance, bool a_reverse = true);
    void loadRecalageZones();
    bool setZonePose(const recalage::Zone& a_zone);
    bool runZone(const recalage::Zone& a_zone);
    bool runStep(const recalage::Step& a_step);
    bool alignInPlace(Angle a_angle, rclcpp::Duration a_timeout);
    bool waitForWallContact(rclcpp::Duration a_timeout);
    bool callCorrectPose(krabi_msgs::srv::CorrectPose::Request::SharedPtr a_request);
    void updateMotorsCurrent(krabi_msgs::msg::MotorsCurrent::SharedPtr a_msg);
    void declareRecalageParameters();

    // ---- Actions avant match (start_recalage_zone / start_recalage / go_to_start) ----
    enum PreMatchRequest
    {
        PRE_MATCH_NONE = 0,
        PRE_MATCH_RECALAGE,
        PRE_MATCH_GO_TO_START
    };
    bool requestPreMatchAction(PreMatchRequest a_request,
                               const std::string& a_zone,
                               std::string& a_message);
    bool handlePreMatchRequest();
    bool preMatchFail(const std::string& a_reason);
    std::string preMatchState() const;
    void publishPreMatchState();
    void repeatPreMatchState();
    void publishRecalageZones();
    bool setPreMatchMotion(bool a_enable);
    bool goToStartPose();
    bool goToPosition(double a_x, double a_y);
    void stopPreMatchMotion();
    bool sleepUnlessMatch(double a_seconds);
    bool getOdomStamp(rclcpp::Time& a_stamp);
    void checkStopMatch();
    bool checkFunnyAction();

    void updateCurrentPose();
    void updateRemainingTime(builtin_interfaces::msg::Duration::SharedPtr remainingTime);
    void updateTirette(std_msgs::msg::Bool::SharedPtr tirette);
    void updateVacuum(std_msgs::msg::Float32::SharedPtr vacuum_msg);
    void updateCaissesSides(krabi_msgs::msg::CaissesSides::SharedPtr a_caisses_sides_msg);
#ifdef YEAR_2026
    void grabFlipDropCallback(const std::shared_ptr<krabi_msgs::srv::GrabFlipDrop::Request> request,
                              std::shared_ptr<krabi_msgs::srv::GrabFlipDrop::Response> response);
#endif
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
        m_actuators.updateStepperElevator(stepper_info_msg);
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

    std::atomic<State> m_state{ State::RUN }; // lu par les services, hors du thread de loop()
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
    rclcpp::Subscription<krabi_msgs::msg::CaissesSides>::SharedPtr m_caisses_sides_sub;

#ifdef YEAR_2026
    rclcpp::Service<krabi_msgs::srv::GrabFlipDrop>::SharedPtr m_grab_flip_drop_srv;
#endif

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
    TiretteFilter m_tirette_filter; // insérée (armed) puis retirée (started)
    float m_vacuum_level;
    int m_vacuum_state;
    bool m_all_done_do_funny = false;

    Actuators2025 m_actuators; // also works for 2026

#ifdef YEAR_2025
    std::shared_ptr<Grabi> m_grabi;
#else
    std::shared_ptr<Billig> m_billig;
#endif
    bool m_goal_init_done;
    bool m_at_least_one_carre_fouille_done;
    std::vector<Position> m_potential_other_robots;

    int m_year;

    std::shared_ptr<Servomotor> m_servo_banner;

    std::thread m_running;
    std::atomic<bool> m_stop_publishing{ false }; // stops publishAll() at shutdown

    int override_gear;
    Etape* m_area_funny = nullptr;

    bool m_leftmost_caisse_is_our_side_up = true;
    bool m_leftcenter_caisse_is_our_side_up = true;
    bool m_rightcenter_caisse_is_our_side_up = false;
    bool m_rightmost_caisse_is_our_side_up = false;
    bool m_funny_action_Y_hysteresis = false;
    bool m_funny_action_X_hysteresis = false;

    bool m_transform_found = false;
    bool m_thermometer_orient_timeout = false;
    bool m_thermometer_move_timeout = false;
    bool m_garde_manger_orient_timeout = false;
    bool m_caisse_orient_timeout = false;

    // ---- Recalage automatique sur bordure ----
    struct RecalageParams
    {
        double contact_offset_back; // m, base_link -> face arrière (contact mur)
        double contact_offset_front; // m, base_link -> face avant
        double still_dist;     // m, déplacement max pour considérer le robot immobile
        double still_time;     // s, durée d'immobilité = contact
        double settle_time;    // s, attente après contact avant de corriger
        double timeout;        // s, timeout global de la poussée
        double align_timeout;  // s, timeout de l'alignement
        double service_timeout; // s, attente de la réponse de correct_pose
        double min_push_dist;  // m, déplacement minimal avant d'accepter un contact
        double max_correction_dist;  // m, correction refusée au-delà
        double max_correction_angle; // rad, correction refusée au-delà
        bool use_lidar_loc;    // si vrai : recalage désactivé (AMCL publie map->odom)
    } m_recalage;
    rclcpp::CallbackGroup::SharedPtr m_recalage_cb_group;
    rclcpp::Client<krabi_msgs::srv::CorrectPose>::SharedPtr m_correct_pose_client;
    rclcpp::Subscription<krabi_msgs::msg::MotorsCurrent>::SharedPtr m_motors_current_sub;
    std::atomic<bool> m_left_wheel_stalled{ false };
    std::atomic<bool> m_right_wheel_stalled{ false };

    // ---- Actions avant match ----
    double m_start_pose_x, m_start_pose_y, m_start_pose_theta; // pose de départ (map)
    std::atomic<int> m_pre_match_request{ PRE_MATCH_NONE };
    std::string m_pre_match_zone; // écrit avant m_pre_match_request, lu après
    recalage::Config m_recalage_config;
    std::atomic<bool> m_pre_match_busy{ false };
    // Le chrono de Core a diminué : un match est en cours, même si goal_strat vient de redémarrer
    // (respawn) et attend la tirette
    std::atomic<bool> m_chrono_running{ false };
    double m_last_remaining_s = -1.; // callback remaining_time seulement
    // Utilisés seulement par le thread de loop()
    std::string m_pre_match_step;         // étape en cours
    std::string m_pre_match_error;        // première erreur de l'action en cours
    std::string m_recalage_done;          // libellé de la zone du dernier recalage réussi
    std::string m_recalage_failure;       // cause d'échec du dernier recalage
    std::string m_go_to_start_failure;    // cause d'échec du dernier go_to_start
    std::mutex m_pre_match_state_mutex;
    std::string m_pre_match_state_sent;   // dernier état publié (protégé par le mutex)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pre_match_state_pub;
    rclcpp::Publisher<krabi_msgs::msg::RecalageZones>::SharedPtr m_recalage_zones_pub;
    rclcpp::Service<krabi_msgs::srv::StartRecalage>::SharedPtr m_start_recalage_zone_srv;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_recalage_sub; // bouton de l'IHM
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_start_recalage_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_go_to_start_srv;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_pre_match_motion_client;
};
