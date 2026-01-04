#include "goal_strategy/goal.h"
#include "goal_strategy/subscriptionCreator.h"
#include <std_msgs/msg/float32.hpp>

// #include <std_msgs/msg/uint16.hpp>

#define goal_MAX_ALLOWED_ANGULAR_SPEED 1.f // rad/s
using namespace std::chrono_literals;

/// ################ Movement methods ################

/**
 * @brief Send cmd to clamp the robot to its position (used in 2021 for manches a air)
 */
void GoalStrat::clamp_mode()
{
    m_strat_mvnt.max_speed.angular.z = 0;
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::CLAMP_MODE;
}

/**
 * @brief Send cmd to disable angular and over current per motor
 */
void GoalStrat::recalage_bordure()
{
    m_strat_mvnt.max_speed.angular.z = 0;
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::RECALAGE_BORDURE;
}

bool GoalStrat::recule(rclcpp::Duration a_time)
{
    return reculeOuAvance(a_time, Distance(1000), true /*recule*/);
}

bool GoalStrat::recule(rclcpp::Duration a_time, Distance a_distance)
{
    return reculeOuAvance(a_time, a_distance, true /*recule*/);
}

bool GoalStrat::avance(rclcpp::Duration a_time)
{
    return reculeOuAvance(a_time, Distance(1000), false /*avance*/);
}

bool GoalStrat::avance(rclcpp::Duration a_time, Distance a_distance)
{
    return reculeOuAvance(a_time, a_distance, false /*avance*/);
}

bool GoalStrat::reculeOuAvance(rclcpp::Duration a_time, Distance a_distance, bool sensRecule)
{
    updateCurrentPose();
    auto l_initial_pose = m_current_pose.getPosition();

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "recule !!");
    recalage_bordure();
    // m_strat_mvnt.reverse_gear = 1;

    if (m_year == 2024)
    {
        sensRecule = !sensRecule;
    }

    override_gear = krabi_msgs::msg::StratMovement::REVERSE;

    if (sensRecule == false)
    {
        override_gear = krabi_msgs::msg::StratMovement::FORWARD;
    }

    publishStratMovement();
    auto recalageTimeoutDeadline = this->now() + a_time;

    Distance l_distance_parcourue = Distance(0);

    while (this->now().seconds() < recalageTimeoutDeadline.seconds()
           && l_distance_parcourue < a_distance)
    {
        // todo fix and reenable
        // rclcpp::spin_some(shared_from_this());
        updateCurrentPose();
        l_distance_parcourue = (l_initial_pose - m_current_pose.getPosition()).getNorme();
        usleep(0.1e6);
    }
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;

    if (this->now().seconds() > recalageTimeoutDeadline.seconds())
    {
        // Timeout reached
        return false;
    }
    return true;
}

bool GoalStrat::reculeDroit(rclcpp::Duration a_time, Distance a_distance)
{
    return reculeOuAvanceDroit(a_time, a_distance, true /*recule*/);
}

bool GoalStrat::avanceDroit(rclcpp::Duration a_time, Distance a_distance)
{
    return reculeOuAvanceDroit(a_time, a_distance, false /*avance*/);
}

bool GoalStrat::reculeOuAvanceDroit(rclcpp::Duration a_time, Distance a_distance, bool sensRecule)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "recule/avance droit !!");

    Position l_position_recule = m_current_pose.getPosition();

    if (sensRecule == false)
    {
        override_gear = krabi_msgs::msg::StratMovement::FORWARD;
        l_position_recule.setX(
          Distance(l_position_recule.getX() + a_distance * cos(m_current_pose.getAngle())));
        l_position_recule.setY(
          Distance(l_position_recule.getY() + a_distance * sin(m_current_pose.getAngle())));
    }
    else
    {
        l_position_recule.setX(
          Distance(l_position_recule.getX() - a_distance * cos(m_current_pose.getAngle())));
        l_position_recule.setY(
          Distance(l_position_recule.getY() - a_distance * sin(m_current_pose.getAngle())));
    }

    return goToDroit(l_position_recule, a_time, a_distance, sensRecule, false);
}

bool GoalStrat::goToDroit(Position& a_position,
                          rclcpp::Duration a_time,
                          Distance a_distance,
                          bool sensRecule,
                          bool a_position_precise = true)
{
    updateCurrentPose();
    auto l_initial_pose = m_current_pose.getPosition();

    // recalage_bordure();
    // m_strat_mvnt.reverse_gear = 1;

    if (m_year == 2024) // On recule dans l'autre sens
    {
        sensRecule = !sensRecule;
    }

    override_gear = krabi_msgs::msg::StratMovement::REVERSE;

    m_goal_pose.setPosition(a_position);
    chooseEffector(false);
    publishGoal();

    auto recalageTimeoutDeadline = this->now() + a_time;

    Distance l_distance_parcourue = Distance(0);
    Distance l_distance_to_objective = Distance(0);
    Distance l_epsilon_distance = Distance(0.05);

    while (this->now().seconds() < recalageTimeoutDeadline.seconds()
           && l_distance_parcourue + l_epsilon_distance < a_distance)
    {
        // todo modify this to go backward
        // rclcpp::spin_some(shared_from_this());
        updateCurrentPose();
        l_distance_parcourue = (l_initial_pose - m_current_pose.getPosition()).getNorme();
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                            "distancePacrourue: " << l_distance_parcourue);
        publishStratMovement();

        usleep(0.1e6);

        l_distance_to_objective = (a_position - m_current_pose.getPosition()).getNorme();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "l_distance_to_objective: " << l_distance_to_objective);
        if (a_position_precise && l_distance_to_objective < REACH_DIST)
        {
            // Success!
            break;
        }
    }
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;

    if (this->now().seconds() >= recalageTimeoutDeadline.seconds())
    {
        return false;
    }
    return true;
}
/**
 * @brief Send cmd to disable angular motion
 *
 */
void GoalStrat::stopAngular()
{
    m_strat_mvnt.max_speed.angular.z = 0;
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::STOP_ANGULAR;
}

/**
 * @brief Send cmd to enable angular motion
 *
 */
void GoalStrat::startAngular()
{
    m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::GO_TO_GOALPOSE_POSITION;
}

/**
 * @brief Send cmd to disable linear motion
 *
 */
void GoalStrat::stopLinear()
{
    m_strat_mvnt.max_speed.linear.x = 0;
    // m_strat_mvnt.orient = 1;
}

/**
 * @brief Send cmd to enable linear motion
 *
 */
void GoalStrat::startLinear()
{
    m_strat_mvnt.max_speed.linear.x = 1;
    // m_strat_mvnt.orient = 0;
}

/**
 * @brief Print in the console the current action perfomed
 *
 */
void GoalStrat::printCurrentAction()
{
    if (!m_state_msg_displayed)
    {
        int etapeId = m_strat_graph->getEtapeEnCours()->getNumero();
        Position goal = m_strat_graph->getEtapeEnCours()->getPosition();
        int mission_type = m_strat_graph->getEtapeEnCours()->getEtapeType();
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Etape id: " << etapeId << std::endl);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "goal: " << goal << std::endl);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "type: " << mission_type << std::endl);
        m_state_msg_displayed = true;
    }
}

/**
 * @brief Align the robot with the specified angle
 *
 * @param a_angle angle to align with
 */
void GoalStrat::alignWithAngle(Angle a_angle)
{
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "orientToAngle " << a_angle << std::endl);
    m_goal_pose.setAngle(a_angle);
    Position goal_position = m_strat_graph->getEtapeEnCours()->getPosition();
    m_goal_pose.setX(goal_position.getX());
    m_goal_pose.setY(goal_position.getY());
    geometry_msgs::msg::PoseStamped l_posestamped;
    l_posestamped.pose = m_goal_pose;
    l_posestamped.header.frame_id = "map";
    m_goal_pose_pub->publish(l_posestamped);

    m_strat_mvnt.goal_pose = l_posestamped;
    m_strat_mvnt.header.frame_id = "map";
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::ORIENT_TOWARD_GOALPOSE_ORIENTATION;
    publishStratMovement();
}

/**
 * @brief Check if the robot reached the goal position
 *
 * @return true if the robot center is closer than REACH_DIST
 */
bool GoalStrat::isArrivedAtGoal()
{
    updateCurrentPose();
    m_dist_to_goal
      = (m_current_pose.getPosition() - m_strat_graph->getEtapeEnCours()->getPosition()).getNorme();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                        "current pose: x = "
                          << m_current_pose.getPosition().getX() << ", y = "
                          << m_current_pose.getPosition().getY() << ", etape_en_cours: x = "
                          << m_strat_graph->getEtapeEnCours()->getPosition().getX() << ", y = "
                          << m_strat_graph->getEtapeEnCours()->getPosition().getY() << std::endl
                          << "Distance to objective: " << m_dist_to_goal);

    float l_reach_dist = REACH_DIST;
    if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::STOCK_MATIERE_PREMIERE
        || m_strat_graph->getEtapeEnCours()->getEtapeType()
             == Etape::EtapeType::AIRE_DE_CONSTRUCTION)
    {
        l_reach_dist = m_grabi->getReach();
    }
    if (isParked())
    {
        // l_reach_dist *= 2;
    }

    if (m_dist_to_goal < l_reach_dist)
    {
        return true;
    }
    return false;
}

/**
 * @brief Check if the robot is Aligned with the specified angle
 *
 * @param angle target angle
 * @return true if the angular difference between the goal and angle is smaller than REACH_ANG
 */
bool GoalStrat::isAlignedWithAngle(Angle angle)
{
    // Output a goal relative to the robot
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                        m_current_pose.getPosition().getX() << std::endl);

    // Is the tool on the back?
    Angle toolAngle = m_current_pose.getAngle();
    if (m_strat_mvnt.reverse_gear == krabi_msgs::msg::StratMovement::REVERSE)
    {
        toolAngle += M_PI;
    }

    // Compute angular diff
    Angle angular_error = AngleTools::diffAngle(angle, toolAngle);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                        "isAlignedWithAngle angle: "
                          << angle << " ? current: " << m_current_pose.getAngle()
                          << "angular_error = " << AngleTools::rad2deg(angular_error) << std::endl);

    // When we reached the correct orientation, angularly stop and switch state
    if (abs(angular_error) < REACH_ANG)
    {
        return true;
    }
    return false;
}

/// ################ ROS2 messages ################

/**
 * @brief Publish the cmd to move the robot toward the goal
 *
 */
void GoalStrat::publishGoal()
{

    geometry_msgs::msg::PoseStamped l_posestamped;

    l_posestamped.pose = m_goal_pose;
    l_posestamped.header.frame_id = "map";
    m_goal_pose_pub->publish(l_posestamped);

    m_strat_mvnt.goal_pose = l_posestamped;

    // m_strat_mvnt.orient = 0;

    publishStratMovement();
}

void GoalStrat::publishStratMovement()
{
    chooseGear();
    m_strat_mvnt.goal_pose.pose = m_goal_pose;
    m_strat_mvnt.header.frame_id = "map";
    m_strat_mvnt.header.stamp = this->now();
    m_strat_movement_pub->publish(m_strat_mvnt);
}

/**
 * @brief Update the robot pose and the various transforms
 *
 */
void GoalStrat::updateCurrentPose()
{
    if (m_tf_buffer_ == nullptr)
    { // Too soon, need to call initTF before
        return;
    }
    try
    {
        // auto base_link_id = tf::resolve(rclcpp::this_node::getNamespace(), "base_link"); //1.7
        // Removal of support for tf_prefix
        auto base_link_id = "base_link";
        const auto& transform
          = m_tf_buffer_->lookupTransform("map", base_link_id, tf2::TimePointZero).transform;
        m_current_pose = Pose(transform);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "%s", ex.what());
        // @todo diagnostic
    }

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                        "updateCurrentPose: " << m_current_pose << std::endl);
}

/**
 * @brief Perform the transition between two goals
 *
 */
void GoalStrat::goToNextMission()
{
    if (m_state == State::ALL_DONE)
    {
        return;
    }

    m_moving_timeout_deadline = this->now() + rclcpp::Duration(m_timeout_moving, 0);

    startLinear();
    m_is_first_action = false;

    m_state_msg_displayed = false;

    m_previous_etape_type = m_strat_graph->getEtapeEnCours()->getEtapeType();

    if (!m_action_aborted)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                            "etape type: " << m_previous_etape_type
                                           << ", score before: " << m_score_match);
        // Check if we scored points
        switch (m_previous_etape_type)
        {
        default:
            break;
        }
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                            "score after: " << m_score_match << std::endl);
    }

    m_action_aborted = false;

    int strat_graph_status = m_strat_graph->update();

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "New intermediate: " << m_strat_graph->getEtapeEnCours()->getNumero()
                                            << ", which is a "
                                            << m_strat_graph->getEtapeEnCours()->getEtapeType());
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "New objective: " << m_strat_graph->getGoal()->getNumero() << ", which is a "
                                         << m_strat_graph->getGoal()->getEtapeType());

    // Long actions
    if (false)
    {
        m_moving_timeout_deadline = this->now() + rclcpp::Duration(m_timeout_moving * 3, 0);
    }

    if (strat_graph_status == -1)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Graph status is -1: we're done");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "All goals accomplished");
        m_all_done_do_funny = true;
        m_state = State::ALL_DONE;
        return;
    }

    return;
}

// 2025 banner deployment
void GoalStrat::pushBanner()
{
    reculeDroit(rclcpp::Duration(2, 0), Distance(0.1));
    avance(rclcpp::Duration(2, 0), Distance(0.1));
}

void GoalStrat::initTF()
{
    auto my_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions l_sub_options;
    l_sub_options.callback_group = my_callback_group;

    m_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    /*m_tf_listener_
      = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_, shared_from_this(), true);*/
    m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *m_tf_buffer_,
      shared_from_this(),
      true,
      tf2_ros::DynamicListenerQoS(), // default
      tf2_ros::StaticListenerQoS(),  // default
      l_sub_options);                // to pass CallbackGroupType::Reentrant
    // m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_);
}

GoalStrat::GoalStrat()
  : Node("goal_strat")
//: m_tf_listener(m_tf_buffer)
{
    std::cout << "coucou from GoalStrat" << std::endl;

    m_tf_buffer_ = nullptr;

    m_goal_init_done = false;
    m_at_least_one_carre_fouille_done = false;
    m_remainig_time = rclcpp::Duration(85, 0);
    m_strat_mvnt.max_speed_at_arrival = 0.0f;
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::GO_TO_GOALPOSE_POSITION;
    m_strat_mvnt.max_speed.linear.x = 0.5f;
    m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
    m_strat_mvnt.reverse_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE; // don't care

    m_state = State::INIT;
    m_tirette = false;
    m_vacuum_level = 0.f;
    m_vacuum_state = OPEN_AIR;

    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE; // don't care

    publisherCreator(
      m_is_blue_pub, m_goal_pose_pub, m_debug_ma_etapes_pub, m_strat_movement_pub, this);

    auto my_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions l_sub_options;
    l_sub_options.callback_group = my_callback_group;

    std::string actuators_name = "actuators_msg";

#ifdef YEAR_2025
    m_year = 2025;
#else
    m_year = 2026;
#endif

    // 2025

    auto l_servo_grabi_left_most = std::make_shared<Servomotor>(10, 67);
    auto l_servo_grabi_center_left = std::make_shared<Servomotor>(10, 52);
    auto l_servo_grabi_center_right = std::make_shared<Servomotor>(10, 67);
    auto l_servo_grabi_right_most = std::make_shared<Servomotor>(10, 62);
    auto l_additionnal_servo = std::make_shared<Servomotor>(10, 62);
    auto l_pump_plank = std::make_shared<Pump>(false, true);
    auto l_ax12_1 = std::make_shared<AX12>(10, 400);
    auto l_ax12_2 = std::make_shared<AX12>(10, 320);
    auto l_ax12_3 = std::make_shared<AX12>(10, 828);
    auto l_ax12_4 = std::make_shared<AX12>(10, 90);
    auto l_servo_grabi_lever = std::make_shared<Servomotor>(10, 175);
    m_servo_banner = std::make_shared<Servomotor>(10, 90);
    auto l_stepper_grabi_elevator = std::make_shared<StepperElevator>(
      100 /* mm/s */, 100 /* mm/s2 */, 100 /* x50mA */, 300 /*mm de haut max*/);

    m_grabi = std::make_shared<Grabi>(Position(Eigen::Vector2d(0.191767f, 0.f)),
                                      Position(Eigen::Vector2d(0.08f, 0.f)),
                                      l_servo_grabi_left_most,
                                      l_servo_grabi_center_left,
                                      l_servo_grabi_center_right,
                                      l_servo_grabi_right_most,
                                      l_ax12_1,
                                      l_ax12_2,
                                      l_ax12_3,
                                      l_servo_grabi_lever,
                                      l_stepper_grabi_elevator,
                                      l_pump_plank);

    m_actuators2025 = Actuators2025(rclcpp::Node::SharedPtr(this),
                                    "actuators2025",
                                    l_servo_grabi_left_most,
                                    l_servo_grabi_center_left,
                                    l_servo_grabi_center_right,
                                    l_servo_grabi_right_most,
                                    l_servo_grabi_lever,
                                    l_additionnal_servo,
                                    l_additionnal_servo,
                                    l_additionnal_servo,
                                    l_ax12_1,
                                    l_ax12_2,
                                    l_ax12_3,
                                    l_ax12_4,
                                    l_stepper_grabi_elevator,
                                    l_pump_plank);

    m_actuators2025.start();

    m_actuators2025.set_score(42);

    init(); // to do before subscribing to /remaining_time

    create_subscriptions(m_remaining_time_match_sub,
                         std::bind(&GoalStrat::updateRemainingTime, this, std::placeholders::_1),
                         m_tirette_sub,
                         std::bind(&GoalStrat::updateTirette, this, std::placeholders::_1),
                         m_vacuum_sub,
                         std::bind(&GoalStrat::updateVacuum, this, std::placeholders::_1),
                         m_other_robots_sub,
                         std::bind(&GoalStrat::updateOtherRobots, this, std::placeholders::_1),
                         m_elevator_sub,
                         std::bind(&GoalStrat::updateStepperElevator, this, std::placeholders::_1),
                         m_ax12_1_info_sub,
                         std::bind(&GoalStrat::updateAX12Info1, this, std::placeholders::_1),
                         m_ax12_2_info_sub,
                         std::bind(&GoalStrat::updateAX12Info2, this, std::placeholders::_1),
                         m_ax12_3_info_sub,
                         std::bind(&GoalStrat::updateAX12Info3, this, std::placeholders::_1),
                         m_ax12_4_info_sub,
                         std::bind(&GoalStrat::updateAX12Info4, this, std::placeholders::_1),
                         m_digital_reads_sub,
                         std::bind(&GoalStrat::updateDigitalReads, this, std::placeholders::_1),
                         l_sub_options,
                         this);

    m_timer = this->create_wall_timer(100ms, std::bind(&GoalStrat::loop, this));
}

void GoalStrat::publishIsBlue()
{
    auto isBlueMsg = std_msgs::msg::Bool();
    isBlueMsg.data = m_is_blue;
    m_is_blue_pub->publish(isBlueMsg);
}

void GoalStrat::publishAll()
{
    std::string s = "mainPubAll";

    pthread_setname_np(pthread_self(), s.c_str());
    while (true)
    {
        usleep(50000);
        updateCurrentPose();
        checkFunnyAction();
        checkStopMatch();
        publishScore();
        publishIsBlue();
        // publishGoal();
        publishDebugInfos();
    }
}

void GoalStrat::updateAX12Info(krabi_msgs::msg::AX12Info::SharedPtr a_ax12_msg, uint8_t id)
{
    m_grabi->updateAX12Infos(a_ax12_msg->current_position,
                             a_ax12_msg->present_temperature,
                             a_ax12_msg->present_current,
                             a_ax12_msg->moving,
                             id);
}

void GoalStrat::updateDigitalReads(std_msgs::msg::Byte::SharedPtr digitalReads)
{
    m_grabi->updateCanDetected(digitalReads->data);
}

/**
 * @brief Update the tirette state
 *
 * @param tirette msg
 */
void GoalStrat::updateTirette(std_msgs::msg::Bool::SharedPtr tirette)
{
    m_tirette = tirette->data;
}

/**
 * @brief Update the positions of the potential other robots
 *
 * @param a_potential_other_robots geometry_msgs::msg::PoseArray
 *
 */
void GoalStrat::updateOtherRobots(geometry_msgs::msg::PoseArray::SharedPtr a_potential_other_robots)
{
    m_potential_other_robots.clear();
    for (auto l_potential_robot : a_potential_other_robots->poses)
    {
        m_potential_other_robots.push_back(Pose(l_potential_robot).getPosition());
    }

    m_strat_graph->setDistancesFromRobotsToEtapes(m_potential_other_robots);
}

/**
 * @brief Update the vacuum level
 *
 * @param vacuum_msg the ROS message
 */
void GoalStrat::updateVacuum(std_msgs::msg::Float32::SharedPtr vacuum_msg)
{
    m_vacuum_level = vacuum_msg->data;
    m_vacuum_state = OPEN_AIR;
    if (abs(m_vacuum_level) > 5000) // Pa
    {
        m_vacuum_state = WEAK_VACUUM;
    }
    if (abs(m_vacuum_level) > 10000) // Pa
    {
        m_vacuum_state = STRONG_VACUUM;
    }
}

/**
 * @brief Update the remaining time and perform the time sensitive actions like the funny action and
 * stopping the robot at the end of the match
 *
 * @param a_remaining_time_match
 */
void GoalStrat::updateRemainingTime(
  builtin_interfaces::msg::Duration::SharedPtr a_remaining_time_match)
{
    m_remainig_time = rclcpp::Duration(*a_remaining_time_match);
    m_strat_graph->setRemainingTime(m_remainig_time.seconds() * 1000);
    checkFunnyAction();
    checkStopMatch();
}

/**
 * @brief Check if the end of the match arrived and if so stopping the robot
 *
 */
void GoalStrat::checkStopMatch()
{
    const rclcpp::Duration stop_timing
      = rclcpp::Duration(0, 0.2 * 10e9); // in seconds before the end of match;

    if (m_remainig_time.seconds() < stop_timing.seconds())
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Match ended, stopping the actuators");
        stopActuators();
        m_state = State::EXIT;
        m_strat_mvnt.max_speed.linear.x = 0;
        m_strat_mvnt.max_speed.angular.z = 0;
    }
}

/**
 * @brief Check if it's time to perform the funny action and perfom it if required
 *
 */
bool GoalStrat::checkFunnyAction()
{
    if (m_all_done_do_funny)
    {
        return true;
    }
    const rclcpp::Duration funny_action_timing = rclcpp::Duration(15, 0); // 10s before T=0;

    if (m_remainig_time.seconds() < funny_action_timing.seconds())
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Doing the funny action");
        return true;
    }
    return false;
}

/**
 * @brief Try to align with an angle during a fix amount of time
 * If the action take more than m_timeout_orient  seconds it is aborted.
 *
 * @param angle to align with
 * @return true
 * @return false
 */
bool GoalStrat::alignWithAngleWithTimeout(Angle angle)
{
    rclcpp::Time orientTimeoutDeadline = this->now() + rclcpp::Duration(m_timeout_orient, 0);

    rclcpp::Rate r(100); // Check at 100Hz the new pose msg
    while (!isAlignedWithAngle(angle) && this->now().seconds() < orientTimeoutDeadline.seconds())
    {
        alignWithAngle(angle);
        m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::ORIENT_TOWARD_GOALPOSE_ORIENTATION;
        // todo fix and reenable
        // rclcpp::spin_some(shared_from_this());
        r.sleep();
    }
    return this->now().seconds() >= orientTimeoutDeadline.seconds();
}

/**
 * @brief Choose the best direction for the robot base on the current and future actions
 *
 */
void GoalStrat::chooseGear()
{
    std_msgs::msg::Bool l_reverseGear;
    if (override_gear != krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE)
    {
        m_strat_mvnt.reverse_gear = override_gear;
        l_reverseGear.data = (override_gear == krabi_msgs::msg::StratMovement::REVERSE);
    }

    else if (
      // 2025
      m_previous_etape_type == Etape::EtapeType::AIRE_DE_CONSTRUCTION)
    {
        l_reverseGear.data = true;
        m_strat_mvnt.reverse_gear = krabi_msgs::msg::StratMovement::REVERSE;
    }
    else if (
      // 2025
      m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::STOCK_MATIERE_PREMIERE
      // || m_previous_etape_type == Etape::EtapeType::DEPART // should not be needed
      || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::AIRE_DE_CONSTRUCTION)
    {
        l_reverseGear.data = false;
        m_strat_mvnt.reverse_gear = krabi_msgs::msg::StratMovement::FORWARD;
    }
    else
    {
        // // Don't care
        // l_reverseGear.data = false;
        m_strat_mvnt.reverse_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
        if (m_year == 2024)
        {
            l_reverseGear.data = true;
            m_strat_mvnt.reverse_gear = krabi_msgs::msg::StratMovement::REVERSE;
        }
    }

    // l_reverseGear.data = true;
    // m_strat_mvnt.reverse_gear = 1;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                        "currentEtapeType = " << m_strat_graph->getEtapeEnCours()->getEtapeType()
                                              << "m_previous_etape_type = " << m_previous_etape_type
                                              << ", reverseGear = " << l_reverseGear.data
                                              << std::endl);
}

/**
 * @brief Abort the action that was currently performed and update the graph to indicate a danger
 * for that action
 *
 */
void GoalStrat::abortAction()
{
    m_strat_graph->collisionAvoided();
    m_action_aborted = true;
}

bool GoalStrat::isParked()
{

    std::vector<Position> l_valid_end_locations;
    if (m_year == 2022)
    {
        l_valid_end_locations.push_back(m_strat_graph->positionCAbsolute(0.975f, 1.375f));
        l_valid_end_locations.push_back(m_strat_graph->positionCAbsolute(0.3, 0.7f));
        l_valid_end_locations.push_back(m_strat_graph->positionCAbsolute(3 - 0.975f, 1.375f));
    }
    else if (m_year == 2025 || m_year == 2026)
    {
        // Only one spot in 2025
        l_valid_end_locations.push_back(m_strat_graph->getParkedPosition());
    }

    for (auto l_position : l_valid_end_locations)
    {
        if ((m_current_pose.getPosition() - l_position).getNorme() < 0.5f)
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief Publish the score
 *
 */
void GoalStrat::publishScore()
{
    int l_score = m_score_match; // points définitivement marqués

    std_msgs::msg::Int16 l_score_match;
    l_score_match.data = static_cast<uint16_t>(std::ceil(l_score));
    // m_score_pub->publish(l_score_match);

    // Is the robot in the right area at the end?
    if (m_remainig_time < rclcpp::Duration(4, 0))
    {
        if (isParked())
        {
            l_score += 10;
        }
        if (m_year == 2023)
        {
            l_score += 5; // deguisement
        }
    }

    if (m_remainig_time.seconds() > 0.0f)
    {
        m_score_match_at_end = l_score;
    }

    m_actuators2025.set_score(m_score_match_at_end);
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
    //                    " score at end " << m_score_match_at_end << std::endl);
}

/**
 * @brief Publish command to disable actuator
 *
 */
void GoalStrat::stopActuators()
{
}

/**
 * @brief Choose the max speed at the end of the current action based on the action type
 *
 */
void GoalStrat::setMaxSpeedAtArrival()
{
    if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::POINT_PASSAGE)
    {
        // No need for complete stop at intermediate stops
        m_strat_mvnt.max_speed_at_arrival = 0.1f; // 0.1f;
    }
    else
    {
        // By default, brake before arriving
        m_strat_mvnt.max_speed_at_arrival = 0.f;
    }
}

void GoalStrat::chooseEffector(bool enable)
{
    // m_strat_mvnt.endpoint_frame_id = tf::resolve(rclcpp::this_node::getNamespace(), "base_link");
    // //1.7 Removal of support for tf_prefix
    m_strat_mvnt.endpoint_frame_id = "base_link";
    if (!enable)
    {
        return;
    }

    if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::AIRE_DE_CONSTRUCTION
        || m_strat_graph->getEtapeEnCours()->getEtapeType()
             == Etape::EtapeType::STOCK_MATIERE_PREMIERE)
    {
        m_strat_mvnt.endpoint_frame_id = "grabi";
    }
}

/**
 * @brief RUN state of the state machine
 *
 * Here is performed most of the functions
 */
void GoalStrat::stateRun()
{
    m_strat_mvnt.max_speed_at_arrival = 0.0f;
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::GO_TO_GOALPOSE_POSITION;
    m_strat_mvnt.max_speed.linear.x = 0.5f;
    m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
    m_strat_mvnt.reverse_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE; // don't care
    updateCurrentPose();

    /*m_strat_mvnt.max_speed_at_arrival = 0.1f;
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::GO_TO_GOALPOSE_POSITION;
    m_strat_mvnt.max_speed.linear.x = 1.f;
    m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
    m_strat_mvnt.reverse_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE; // don't care*/

    chooseEffector();

    publishScore();
    publishDebugInfos();
    bool isLate = false;
    printCurrentAction();

    chooseGear(); // Go in reverse gear if needed
    setMaxSpeedAtArrival();

    if (m_vacuum_state == OPEN_AIR)
    {
        // Count it as dropped, do not attempt to put it in the vitrine
        /*if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::VITRINE)
        {
            m_action_aborted = true;
            goToNextMission();
        }*/
    }
    if (checkFunnyAction())
    {
        const rclcpp::Duration funny_action_timing_2 = rclcpp::Duration(1, 0); // 1s before T=0;

        // runHome
        m_strat_mvnt.max_speed_at_arrival = 0.0f; // Set to 0 if you don't want an overshoot
        m_strat_mvnt.max_speed.linear.x = 1.f;
        m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
        m_strat_mvnt.reverse_gear
          = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE; // don't care
        chooseEffector(false); // We want the center of the robot to be positionned
        // Zone de fouille
        // m_goal_pose.setPosition(m_strat_graph->positionCAbsolute(0.975f, 1.375f));
        // Zone de départ
        // if (m_area_funny == nullptr)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                                "FAIL getting best area for funny action, revert to default");
            m_goal_pose.setPosition(m_strat_graph->getParkedPosition());
        }

        if (isParked() && (m_year == 2024 || m_year == 2025))
        {
            // avoid turning inside the area
            m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::STOP_ANGULAR;
        }
        publishGoal();
        return;
    }

    if (!m_is_first_action && (this->now() >= m_moving_timeout_deadline))
    {
        isLate = true;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "Robot is late (spent more than "
                             << m_timeout_moving << "seconds trying to reach destination)"
                             << std::endl);
    }

    if (isArrivedAtGoal())
    {
        if (isParked())
        {
            // Does not work yet, but not a real problem
            // stopAngular();
        }

        rclcpp::Time recalageTimeoutDeadline;
        bool successAlign = true;
        bool successGoTo = true;
        Position positionDeposeThermometre;

        switch (m_strat_graph->getEtapeEnCours()->getEtapeType())
        {
        case Etape::EtapeType::AIRE_DE_CONSTRUCTION:
            m_score_match += m_strat_graph->dropPlateformes(m_strat_graph->getEtapeEnCours());
            stopLinear();
            stopAngular();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Drop plateforme");
            m_grabi->drop_plateforme();
            startLinear();
            startAngular();
            recule(rclcpp::Duration(4, 0), Distance(0.2));
            stopLinear();
            stopAngular();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Resetting elevator");
            m_grabi->auto_initGrabi(false);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Done resetting elevator");
            startLinear();
            startAngular();
            break;
        case Etape::EtapeType::STOCK_MATIERE_PREMIERE:
            m_strat_graph->grabPlateformes(m_strat_graph->getEtapeEnCours());
            stopLinear();
            stopAngular();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Grab plateforme");
            m_grabi->grab_plateforme();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Done grabbing plateforme");

            while (!m_grabi->allCansDetected())
            {
                // m_strat_graph->getEtapeEnCours()->get;
                //  wiggle
                /*alignWithAngleWithTimeout(Angle((m_goal_pose.getPose()).getAngle()) + 0.1,
                                          "Timeout while orienting");
                alignWithAngleWithTimeout(Angle((m_goal_pose.getPose()).getAngle()) - 0.1,
                                          "Timeout while orienting");*/
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Failed catch all cans :(");
                break;
            }
            startLinear();
            startAngular();
            break;
        case Etape::EtapeType::THERMOMETRE:
#ifdef YEAR_2026
            stopLinear();
            stopAngular();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Grab thermometre");
            m_strat_graph->grabThermometre();

            startAngular();
            successAlign = alignWithAngleWithTimeout(Angle(0));
            if (!successAlign)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                                   "Timeout while orienting toward thermometre");
                // @todo add diagnostic?
            }
            positionDeposeThermometre = Position(Distance(0.8f), Distance(0.7f));

            if (m_is_blue)
            {
                positionDeposeThermometre = Position(Distance(-0.8f), Distance(0.7f));
            }
            startLinear();
            startAngular();

            successGoTo = goToDroit(positionDeposeThermometre,
                                    rclcpp::Duration(3 * m_timeout_orient, 0),
                                    Distance(10000),
                                    true);
            if (!successGoTo)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                                   "Timeout while moving the thermometre");
                // @todo add diagnostic?
            }

            m_strat_graph->dropThermometre();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Done grabbing thermometre");
            startLinear();
            startAngular();
#endif
            break;
        case Etape::EtapeType::GARDE_MANGER:
            m_strat_graph->dropCaisses(m_strat_graph->getEtapeEnCours());
            break;

        case Etape::EtapeType::ZONE_DE_RAMASSAGE:
            m_strat_graph->grabCaisses(m_strat_graph->getEtapeEnCours());
            break;

        default:
            // stopAngular();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "No special action here\n");
            break;
        }
        goToNextMission();
    }
    else if (isLate)
    {
        // If it has been too long
        // Then it means that there is an obstacle on the way
        // The first action is excluded because we wait for the tirette (+ no reason for an
        // opponent + no other way)
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "Timeout, probable obstacle on the way. Trying another path."
                             << std::endl);
        abortAction();
        goToNextMission();
    }
    Position goal_position = m_strat_graph->getEtapeEnCours()->getPosition();
    m_goal_pose.setPosition(goal_position);
    publishGoal();
    m_goal_init_done = true;
}

/**
 * @brief EXIT State of the state machine
 * Stop the actuators before shutting down the node
 */
void GoalStrat::stateExit()
{
    stopActuators();
    publishScore();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Mission finished, turning off actuators");
}

/**
 * @brief Initialization of the node
 *
 */
void GoalStrat::init()
{
    m_funny_action_counted = false;

    if (m_year == 2023)
    {
        m_score_match = 5;   // panier present
        m_score_match += 10; // 10 cerises
        m_score_match += 5;  // comptage correct
    }
    if (m_year == 2024)
    {
        m_score_match += 1; // check if init is correct (substracted from zone de fin )
    }
    if (m_year == 2025)
    {
        m_score_match = 20; // Banner
    }

    m_dist_to_goal = 100.;
    m_state_msg_displayed = false;

    /*************************************************
     *                   Main loop                   *
     *************************************************/

    // Initialize time
    m_timeout_moving = 15; // sec
    m_timeout_orient = 10; // sec
    m_is_first_action = true;

    // Choose side
    this->declare_parameter("isBlue", true);
    m_is_blue = this->get_parameter("isBlue").as_bool();

    if (m_is_blue)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Is Blue !" << std::endl);
    }
    else
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Not Blue :'(" << std::endl);
    }

    m_action_aborted = false;

#ifdef YEAR_2025
    // Choose starting position
    this->declare_parameter("startingPosition", "FRONT_START");
    std::string l_starting_position_string = this->get_parameter("startingPosition").as_string();
    if (m_year == 2025)
    {
        m_starting_position_2025 = FRONT_START;
        if (l_starting_position_string == "FRONT")
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "We start on the front" << std::endl);
            m_starting_position_2025 = FRONT_START;
        }
        else if (l_starting_position_string == "COTE")
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                                "We start in the opposite side, in the center" << std::endl);
            m_starting_position_2025 = COTE_START;
        }
        else if ((l_starting_position_string == "PAMI"))
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                                "We start near the pami" << std::endl);
            m_starting_position_2025 = PAMI_START;
        }
        else
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                               "Unkown starting position" << std::endl);
        }
        m_strat_graph = std::make_unique<Coupe2025>(!m_is_blue, m_starting_position_2025);
    }
#else
    m_strat_graph = std::make_unique<Coupe2026>(!m_is_blue);
#endif

    m_strat_graph->update();
    m_previous_etape_type = Etape::EtapeType::POINT_PASSAGE;
    // m_strat_graph->getEtapeEnCours()->getEtapeType(); // Was shitting when the first action was
    // one unidirectionnal
    m_state = State::WAIT_TIRETTE;

    m_running = std::thread(&GoalStrat::publishAll, this);
}

/**
 * @brief State machine that handles the different goals
 *
 * @return int
 */
void GoalStrat::loop()
{
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                       "mon stock: " << m_strat_graph->getStock().size() << std::endl);

    switch (m_state)
    {
    case State::INIT:
        // init(); //init before
        break;
    case State::WAIT_TIRETTE:
        m_grabi->initializeElevator();
        if (m_state == State::WAIT_TIRETTE && m_tirette && m_remainig_time > rclcpp::Duration(1, 0))
        {
            m_state = State::RUN;
            if (m_year == 2025)
            {
                publishDebugInfos();
                pushBanner();
                m_grabi->initGrabi(true);
            }
        }
        break;
    case State::RUN:
        stateRun();
        break;
    case State::ALL_DONE:
        stateRun();
        break;
    case State::EXIT:
        stateExit();
        break;
    }
    // rclcpp::spin_some(shared_from_this());
}

/**
 * @brief Publisher to visually debug the strategy graph
 *
 */
void GoalStrat::publishDebugInfos()
{
    visualization_msgs::msg::MarkerArray ma;
    m_strat_graph->debugEtapes(ma);
    m_debug_ma_etapes_pub->publish(ma);
}

void GoalStrat::stop()
{
    m_strat_mvnt.max_speed.angular.z = 0;
    m_strat_mvnt.max_speed.linear.x = 0;
    publishStratMovement();
}

// Entry point
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalStrat>();
    node->initTF();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
