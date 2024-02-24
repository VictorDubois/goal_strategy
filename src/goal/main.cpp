#include "goal_strategy/goal.h"
#include <std_msgs/msg/float32.hpp>


//#include <std_msgs/msg/uint16.hpp>

#define goal_MAX_ALLOWED_ANGULAR_SPEED 5.f // rad/s

/**
 * @brief Set the arm to a specified position
 *
 * @param position FOLDED, IN, OUT, UP, RELEASE, DOWN
 */
void GoalStrat::moveArm(enum PositionServo position)
{
    switch (position)
    {
    case FOLDED:
        m_servos_cmd.s3_speed = 128;
        m_servos_cmd.s3_angle = 155;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Actually fold servo" << std::endl);
        break;
    case IN:
        m_servos_cmd.s3_speed = 255;
        m_servos_cmd.s3_angle = 155;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Actually move servo IN" << std::endl);
        break;
    case OUT:
        m_servos_cmd.s3_speed = 255;
        m_servos_cmd.s3_angle = 23;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Actually move servo OUT" << std::endl);
        break;
    case UP:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 148;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Actually move servo UP" << std::endl);
        break;
    case RELEASE:
        m_servos_cmd.s3_speed = 128;
        m_servos_cmd.s3_angle = 155;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Actually release servo" << std::endl);
        break;
    case DOWN:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 42;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Actually move servo DOWN" << std::endl);
        break;
    default:
        break;
    }
    m_servos_cmd.enable = true;
    // m_arm_servo_pub->publish(m_servos_cmd);
}

/**
 * @brief Send cmd to clamp the robot to its position (used in 2021 for manches a air)
 */
void GoalStrat::clamp_mode()
{
    m_strat_mvnt.max_speed.angular.z = 0;
    m_strat_mvnt.orient = 3;
}

/**
 * @brief Send cmd to disable angular and over current per motor
 */
void GoalStrat::recalage_bordure()
{
    m_strat_mvnt.max_speed.angular.z = 0;
    m_strat_mvnt.orient = 2;
}

void GoalStrat::recule(rclcpp::Duration a_time)
{
    recule(a_time, Distance(1000));
}
void GoalStrat::recule(rclcpp::Duration a_time, Distance a_distance)
{
    updateCurrentPose();
    auto l_initial_pose = m_current_pose.getPosition();

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "recule !!");
    recalage_bordure();
    //m_strat_mvnt.reverse_gear = 1;
    override_gear = 1;

    publishStratMovement();
    auto recalageTimeoutDeadline = m_node->now() + a_time;

    Distance l_distance_parcourue = Distance(0);

    while (m_node->now().seconds() < recalageTimeoutDeadline.seconds() && l_distance_parcourue < a_distance)
    {
        rclcpp::spin_some(m_node);
        updateCurrentPose();
        l_distance_parcourue = (l_initial_pose - m_current_pose.getPosition()).getNorme();
        usleep(0.1e6);
    }
    override_gear = 2;
}
void GoalStrat::reculeDroit(rclcpp::Duration a_time, Distance a_distance)
{
    updateCurrentPose();
    auto l_initial_pose = m_current_pose.getPosition();

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "recule !!");
    //recalage_bordure();
    //m_strat_mvnt.reverse_gear = 1;
    override_gear = 1;

    Position l_position_recule = m_current_pose.getPosition();
    l_position_recule.setX(Distance(l_position_recule.getX()
                           - a_distance * cos(m_current_pose.getAngle())));
    l_position_recule.setY(Distance(l_position_recule.getY()
                           - a_distance * sin(m_current_pose.getAngle())));

    m_goal_pose.setPosition(l_position_recule);

    chooseEffector(false);
    publishStratMovement();
    
    auto recalageTimeoutDeadline = m_node->now() + a_time;

    Distance l_distance_parcourue = Distance(0);

    while (m_node->now().seconds() < recalageTimeoutDeadline.seconds() && l_distance_parcourue < a_distance)
    {
        rclcpp::spin_some(m_node);
        updateCurrentPose();
        l_distance_parcourue = (l_initial_pose - m_current_pose.getPosition()).getNorme();
        usleep(0.1e6);
    }
    override_gear = 2;
}
/**
 * @brief Send cmd to disable angular motion
 *
 */
void GoalStrat::stopAngular()
{
    m_strat_mvnt.max_speed.angular.z = 0;
    m_strat_mvnt.orient = 4;
}

/**
 * @brief Send cmd to enable angular motion
 *
 */
void GoalStrat::startAngular()
{
    m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
    m_strat_mvnt.orient = 0;
}

/**
 * @brief Send cmd to disable linear motion
 *
 */
void GoalStrat::stopLinear()
{
    m_strat_mvnt.max_speed.linear.x = 0;
    //m_strat_mvnt.orient = 1;
}

/**
 * @brief Send cmd to enable linear motion
 *
 */
void GoalStrat::startLinear()
{
    m_strat_mvnt.max_speed.linear.x = 1;
    //m_strat_mvnt.orient = 0;
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
    m_strat_mvnt.orient = 1;
    publishStratMovement();
}

bool GoalStrat::isArrivedAtGoal()
{
    return isArrivedAtGoal(Distance(0));
}

/**
 * @brief Check if the robot reached the goal position
 *
 * @return true if the robot center is closer than REACH_DIST
 */
bool GoalStrat::isArrivedAtGoal(Distance a_offset)
{
    updateCurrentPose();
    m_dist_to_goal
      = (m_current_pose.getPosition() - m_strat_graph->getEtapeEnCours()->getPosition()).getNorme();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "current pose: x = "
                     << m_current_pose.getPosition().getX()
                     << ", y = " << m_current_pose.getPosition().getY() << ", etape_en_cours: x = "
                     << m_strat_graph->getEtapeEnCours()->getPosition().getX() << ", y = "
                     << m_strat_graph->getEtapeEnCours()->getPosition().getY() << std::endl
                     << "Distance to objective: " << m_dist_to_goal);

    float l_reach_dist = REACH_DIST;
    if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::BOUEE
        || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::HEXAGON_PLAT)
    {
        l_reach_dist = m_theThing->getReach();
    }
    if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PILE_GATEAU
        || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::ASSIETTE)
    {
        l_reach_dist = m_claws->getReach();
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
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), m_current_pose.getPosition().getX() << std::endl);

    // Is the tool on the back?
    Angle toolAngle = m_current_pose.getAngle();
    if (m_strat_mvnt.reverse_gear == 1)
    {
        toolAngle += M_PI;
    }

    // Compute angular diff
    Angle angular_error = AngleTools::diffAngle(angle, toolAngle);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "isAlignedWithAngle angle: "
                     << angle << " ? current: " << m_current_pose.getAngle()
                     << "angular_error = " << AngleTools::rad2deg(angular_error) << std::endl);

    // When we reached the correct orientation, angularly stop and switch state
    if (abs(angular_error) < REACH_ANG)
    {
        return true;
    }
    return false;
}

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
    
    //m_strat_mvnt.orient = 0;

    publishStratMovement();
}

void GoalStrat::publishStratMovement()
{
    chooseGear();
    m_strat_mvnt.goal_pose.pose = m_goal_pose;
    m_strat_mvnt.header.frame_id = "map";
    m_strat_mvnt.header.stamp =m_node->now();
    m_strat_movement_pub->publish(m_strat_mvnt);
}


/**
 * @brief Update the robot pose and the various transforms
 *
 */
void GoalStrat::updateCurrentPose()
{
    try
    {
        //auto base_link_id = tf::resolve(rclcpp::this_node::getNamespace(), "base_link"); //1.7 Removal of support for tf_prefix
        auto base_link_id = "base_link";
        const auto& transform
          = m_tf_buffer.lookupTransform("map", base_link_id, rclcpp::Time(0)).transform;
        m_current_pose = Pose(transform);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "%s", ex.what());
    }

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "updateCurrentPose: " << m_current_pose << std::endl);
}

/**
 * @brief Perform the transition between two goals
 *
 */
void GoalStrat::goToNextMission()
{
    m_servo_out = false;

    m_moving_timeout_deadline = m_node->now() + rclcpp::Duration(m_timeout_moving, 0);

    startLinear();
    m_is_first_action = false;

    m_state_msg_displayed = false;

    m_previous_etape_type = m_strat_graph->getEtapeEnCours()->getEtapeType();

    if (!m_action_aborted)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "etape type: " << m_previous_etape_type
                                        << ", score before: " << m_score_match);
        // Check if we scored points
        switch (m_previous_etape_type)
        {
        case Etape::EtapeType::CARRE_FOUILLE:
            m_score_match += 3; // 3 carres valides sur 5 rapportent 5 points
            if (!m_at_least_one_carre_fouille_done)
            {
                m_score_match += 5;
                m_score_match += 2; // first is always good
            }
            m_at_least_one_carre_fouille_done = true;
            break;
        case Etape::EtapeType::STATUETTE:
            // m_score_match += 10;//replique
            m_score_match += 5; // statuette gone
            m_strat_graph->catchStatuette();
            break;
        case Etape::EtapeType::VITRINE:
            m_score_match += 10; // drop statuette
            m_score_match += 10; // allume vitrine
            m_strat_graph->dropStatuette();
            break;
        case Etape::EtapeType::PHARE:
            m_score_match += 3;
            // Will the phare have enough time to be raised?
            if (m_remainig_time.seconds() > 10.)
            {
                m_score_match += 10;
            }
            break;
        case Etape::EtapeType::PORT:
            switch (m_strat_graph->getEtapeEnCours()->getNumero())
            {
            case 10:
                m_score_match += 3;
                break;
            case 11:
                m_score_match += 3;
                break;
            case 17:
                m_score_match += 6;
                break;
            default:
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Warning, unkown port!" << std::endl);
                m_score_match += 2;
                break;
            }
            break;
        case Etape::EtapeType::MANCHE_A_AIR:
            // 5 for the first, 10 for the second
            m_score_match += 5;
            if (m_first_manche_a_air_done)
            {
                m_score_match += 5;
            }
            m_first_manche_a_air_done = true;
            break;

        default:
            break;
        }
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "score after: " << m_score_match << std::endl);
    }

    m_action_aborted = false;

    int strat_graph_status = m_strat_graph->update();

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New intermediate: " << m_strat_graph->getEtapeEnCours()->getNumero()
                                         << ", which is a "
                                         << m_strat_graph->getEtapeEnCours()->getEtapeType());
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New objective: " << m_strat_graph->getGoal()->getNumero() << ", which is a "
                                      << m_strat_graph->getGoal()->getEtapeType());

    // Long actions
    if (false)
    {
        m_moving_timeout_deadline = m_node->now() + rclcpp::Duration(m_timeout_moving * 3, 0);
    }

    if (strat_graph_status == -1)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Graph status is -1: we're done");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "All goals accomplished");
        m_state = State::EXIT;
        return;
    }

    if (!m_claws_openned_once && m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PILE_GATEAU)
    {
        m_claws->release_pile(false);
        m_claws_openned_once = true;
    }

    return;
}

void GoalStrat::pushCarreFouille()
{
    m_servo_pusher->setAngle(130);
    usleep(0.6e6); // takes 330ms, with a x2 margin
}

void GoalStrat::retractePusher()
{
    //m_servo_pusher->setAngle(82);
    //usleep(0.6e6); // takes 330ms, with a x2 margin
}

void GoalStrat::dropCherries()
{
    m_servo_cherries->setAngle(110);
    usleep(5.0e6);
}

void GoalStrat::closeCherriesDispenser()
{
    m_servo_cherries->setAngle(140);
    //usleep(1.0e6);
}

GoalStrat::GoalStrat()
  : m_tf_listener(m_tf_buffer)
{
    m_goal_init_done = false;
    m_at_least_one_carre_fouille_done = false;
    m_remainig_time = rclcpp::Duration(100, 0);
    m_strat_mvnt.max_speed_at_arrival = 0.0f;
    m_strat_mvnt.orient = 0;
    m_strat_mvnt.max_speed.linear.x = 0.5f;
    m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
    m_strat_mvnt.reverse_gear = 2; // don't care

    m_state = State::INIT;
    m_tirette = false;
    m_vacuum_level = 0.f;
    m_vacuum_state = OPEN_AIR;

    override_gear = 2; // don't care

    m_goal_pose_pub = m_node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1000);
    m_arm_servo_pub = m_node->create_publisher<krabi_msgs::msg::ServosCmd>("cmd_servos", 1000);
    m_debug_ma_etapes_pub = m_node->create_publisher<visualization_msgs::msg::MarkerArray>("debug_etapes", 5);
    m_strat_movement_pub = m_node->create_publisher<krabi_msgs::msg::StratMovement>("strat_movement", 5);

    m_remaining_time_match_sub
      = m_node->create_subscription("/remaining_time", 5, &GoalStrat::updateRemainingTime, this);
    m_tirette_sub = m_node->create_subscription("tirette", 5, &GoalStrat::updateTirette, this);
    m_vacuum_sub = m_node->create_subscription("vacuum", 5, &GoalStrat::updateVacuum, this);
    m_other_robots_sub
      = m_node->create_subscription("dynamic_obstacles", 5, &GoalStrat::updateOtherRobots, this);

    std::string actuators_name = "actuators_msg";

    m_year = 2023;

    m_servo_pusher = std::make_shared<Servomotor>(255, 75);
    auto l_pump_arm = std::make_shared<Pump>(false, true);
    auto l_fake_statuette_pump = std::make_shared<Pump>(false, true);
    auto l_servo_arm_base = std::make_shared<Servomotor>(10, 40);
    auto l_servo_arm_mid = std::make_shared<Servomotor>(10, 25);
    auto l_servo_arm_suction_cup = std::make_shared<Servomotor>(10, 90);

    m_theThing = std::make_shared<Grabber>(Position(Eigen::Vector2d(0.0, -0.5)),
                                           l_servo_arm_base,
                                           l_servo_arm_mid,
                                           l_servo_arm_suction_cup,
                                           l_pump_arm);

    m_actuators = Actuators(m_node,
                            actuators_name + "old",
                            m_servo_pusher,
                            l_fake_statuette_pump,
                            l_servo_arm_base,
                            l_servo_arm_mid,
                            l_servo_arm_suction_cup,
                            l_pump_arm);

    if (m_year == 2022)
    {
        m_actuators.start();
        retractePusher();
        m_theThing->release_statuette();
    }


    while (false) // Test the grabber
    {
        m_theThing->grab_statuette();
        pushCarreFouille();

        m_theThing->release_statuette();
        retractePusher();
    }

    m_servo_cherries = std::make_shared<Servomotor>(255, 75);
    auto l_servo_left_claw = std::make_shared<Servomotor>(10, 90);
    auto l_servo_right_claw = std::make_shared<Servomotor>(10, 90);

    m_actuators2023 = Actuators2023(
      m_node, actuators_name, m_servo_cherries, l_servo_left_claw, l_servo_right_claw, l_servo_arm_base, l_servo_arm_mid, l_servo_arm_suction_cup, l_pump_arm);

    if (m_year == 2023)
    {
        m_actuators2023.start();
    }

    m_claws = std::make_shared<Claws>(
      Position(Eigen::Vector2d(0.32f, 0.f)),Position(Eigen::Vector2d(0.2f, 0.f)), l_servo_left_claw, l_servo_right_claw);

    m_claws->setInside();
    /* servo check */
    /*m_claws->retract();
    usleep(1000000);
    m_claws->release_pile();
    usleep(2000000);
    m_claws->grab_pile();
    usleep(2000000);*/

    /* servo init */
    m_claws->retract(false);

    closeCherriesDispenser();
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
        m_arm_servo_pub->publish(m_servos_cmd);
        if (m_goal_init_done)
        {
            //publishGoal();
            publishDebugInfos();
        }
    }
}

/**
 * @brief Update the tirette state
 *
 * @param tirette msg
 */
void GoalStrat::updateTirette(std_msgs::msg::Bool tirette)
{
    m_tirette = tirette.data;
}

/**
 * @brief Update the positions of the potential other robots
 *
 * @param a_potential_other_robots geometry_msgs::msg::PoseArray
 *
 */
void GoalStrat::updateOtherRobots(geometry_msgs::msg::PoseArray a_potential_other_robots)
{
    m_potential_other_robots.clear();
    for (auto l_potential_robot : a_potential_other_robots.poses)
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
void GoalStrat::updateVacuum(std_msgs::msg::Float32 vacuum_msg)
{
    m_vacuum_level = vacuum_msg.data;
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
void GoalStrat::updateRemainingTime(builtin_msgs::msg::Duration a_remaining_time_match)
{
    m_remainig_time = a_remaining_time_match.data;
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
    const rclcpp::Duration stop_timing = rclcpp::Duration(0, 0.2 * 10e9); // in seconds before the end of match;

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
    const rclcpp::Duration funny_action_timing = rclcpp::Duration(10, 0); // 10s before T=0;

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
    rclcpp::Time orientTimeoutDeadline = m_node->now() + rclcpp::Duration(m_timeout_orient);

    rclcpp::Rate r(100); // Check at 100Hz the new pose msg
    while (!isAlignedWithAngle(angle) && m_node->now().seconds() < orientTimeoutDeadline.seconds())
    {
        alignWithAngle(angle);
        m_strat_mvnt.orient = 1;
        rclcpp::spin_some(m_node);
        r.sleep();
    }
    return m_node->now().seconds() >= orientTimeoutDeadline.seconds();
}

/**
 * @brief Choose the best direction for the robot base on the current and future actions
 *
 */
void GoalStrat::chooseGear()
{
    std_msgs::msg::Bool l_reverseGear;
    if (override_gear != 2)
    {
        m_strat_mvnt.reverse_gear = override_gear;
        l_reverseGear.data = (override_gear == 1);
    }
    
    else if (m_previous_etape_type == Etape::EtapeType::MANCHE_A_AIR
        || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PHARE
        || m_previous_etape_type == Etape::EtapeType::PORT
        || m_previous_etape_type == Etape::EtapeType::STATUETTE
        || m_previous_etape_type == Etape::EtapeType::VITRINE
        || m_previous_etape_type == Etape::EtapeType::ASSIETTE
        || (m_previous_etape_type == Etape::EtapeType::PILE_GATEAU && m_strat_graph->getEtapeEnCours()->getEtapeType() != Etape::EtapeType::ASSIETTE)
        || (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::CARRE_FOUILLE
            && m_is_blue))
    {
        l_reverseGear.data = true;
        m_strat_mvnt.reverse_gear = 1;
    }
    else if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::MANCHE_A_AIR
             || m_previous_etape_type == Etape::EtapeType::PHARE
             || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PILE_GATEAU
             || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::ASSIETTE
             || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PORT
             || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::STATUETTE
             || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::VITRINE
             || (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::CARRE_FOUILLE
                 && !m_is_blue))
    {
        l_reverseGear.data = false;
        m_strat_mvnt.reverse_gear = 0;
    }
    else
    {
        // Don't care
        l_reverseGear.data = false;
        m_strat_mvnt.reverse_gear = 2;
    }

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "currentEtapeType = " << m_strat_graph->getEtapeEnCours()->getEtapeType()
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
    if (m_remainig_time < rclcpp::Duration(4,0))
    {
        l_score += 5; // deguisement
        std::vector<Position> l_valid_end_locations;
        if (m_year == 2022)
        {
            l_valid_end_locations.push_back(m_strat_graph->positionCAbsolute(0.975f, 1.375f));
            l_valid_end_locations.push_back(m_strat_graph->positionCAbsolute(0.3, 0.7f));
            l_valid_end_locations.push_back(m_strat_graph->positionCAbsolute(3 - 0.975f, 1.375f));
        }
        else if (m_year == 2023)
        {
            // Auto add all of our assiettes as valid end locations
            for (auto& etape : Etape::getTableauEtapesTotal())
            {
                if (etape)
                {
                    if (etape->getEtapeType() == Etape::EtapeType::ASSIETTE)
                    {
                        auto assiette = static_cast<Assiette&>(*(etape->getAction()));
                        if (assiette.getOwner() == Owner::us)
                        {
                            l_valid_end_locations.push_back(etape->getPosition());
                        }
                    }
                }
            }
        }

        for (auto l_position : l_valid_end_locations)
        {
            if ((m_current_pose.getPosition() - l_position).getNorme() < 0.3f)
            {
                l_score += 15;
            }
        }
    }

    if (m_remainig_time.seconds() > 0.0f)
    {
        m_score_match_at_end = l_score;
    }

    m_actuators.set_score(m_score_match_at_end);
    m_actuators2023.set_score(m_score_match_at_end);
    
}

/**
 * @brief Publish command to disable actuator
 *
 */
void GoalStrat::stopActuators()
{
    m_servos_cmd.enable = false;
    m_arm_servo_pub->publish(m_servos_cmd);

    m_actuators2023.disguise();
    m_actuators.shutdown();
    m_actuators2023.shutdown();
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
        m_strat_mvnt.max_speed_at_arrival = 0.1f;//0.1f;
    }
    else
    {
        // By default, brake before arriving
        m_strat_mvnt.max_speed_at_arrival = 0.f;
    }
}

void GoalStrat::chooseEffector(bool enable)
{
    //m_strat_mvnt.endpoint_frame_id = tf::resolve(rclcpp::this_node::getNamespace(), "base_link"); //1.7 Removal of support for tf_prefix
    m_strat_mvnt.endpoint_frame_id = "base_link";
    if (!enable)
    {
        return;
    }

    if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::BOUEE)
    {
        //m_strat_mvnt.endpoint_frame_id = tf::resolve(rclcpp::this_node::getNamespace(), "suction_cup"); //1.7 Removal of support for tf_prefix
        m_strat_mvnt.endpoint_frame_id = "suction_cup";
    }
    if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::ASSIETTE ||
        m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PILE_GATEAU )
    {
        //m_strat_mvnt.endpoint_frame_id = tf::resolve(rclcpp::this_node::getNamespace(), "claws_inside"); //1.7 Removal of support for tf_prefix
        m_strat_mvnt.endpoint_frame_id = "claws_inside";
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
    m_strat_mvnt.orient = 0;
    m_strat_mvnt.max_speed.linear.x = 0.5f;
    m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
    m_strat_mvnt.reverse_gear = 2; // don't care
    updateCurrentPose();

    /*m_strat_mvnt.max_speed_at_arrival = 0.1f;
    m_strat_mvnt.orient = 0;
    m_strat_mvnt.max_speed.linear.x = 1.f;
    m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
    m_strat_mvnt.reverse_gear = 2; // don't care*/

    chooseEffector();

    publishScore();
    publishDebugInfos();
    m_arm_servo_pub->publish(m_servos_cmd);
    bool isLate = false;
    printCurrentAction();

    chooseGear(); // Go in reverse gear if needed
    setMaxSpeedAtArrival();

    if (m_vacuum_state == OPEN_AIR)
    {
        m_strat_graph
          ->dropStatuette(); // Count it as dropped, do not attempt to put it in the vitrine
        if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::VITRINE)
        {
            m_action_aborted = true;
            goToNextMission();
        }
    }

    if (checkFunnyAction())
    {
        const rclcpp::Duration funny_action_timing_2 = rclcpp::Duration(1,0); // 1s before T=0;

        if (m_remainig_time.seconds() < funny_action_timing_2.seconds())
        {
            // monte le bras pusher, au cas où on est coincé
            pushCarreFouille();
        }

        // runHome
        m_strat_mvnt.max_speed_at_arrival = 0.1f;
        m_strat_mvnt.max_speed.linear.x = 1.f;
        m_strat_mvnt.max_speed.angular.z = goal_MAX_ALLOWED_ANGULAR_SPEED;
        m_strat_mvnt.reverse_gear = 2; // don't care
        chooseEffector(false); // We want the center of the robot to be positionned
        // Zone de fouille
        // m_goal_pose.setPosition(m_strat_graph->positionCAbsolute(0.975f, 1.375f));
        // Zone de départ
        if(m_assiette_funny == nullptr) // Not initialized yet
        {
            // try to update
            m_assiette_funny = m_strat_graph->getBestAssietteForFunny();
            if (m_assiette_funny != nullptr) // If no error
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "best assiette found for funny: " << m_assiette_funny->getGoalPosition());
            }
        }

        if(m_assiette_funny == nullptr)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "FAIL getting best assiette for funny action, revert to default");
            m_goal_pose.setPosition(m_strat_graph->positionCAbsolute(Distance(1.5f-0.375f), Distance(1.525f)));
        }
        else
        {
            m_goal_pose.setPosition(m_assiette_funny->getGoalPosition());
        }
        publishGoal();
        return;
    }

    // prepare arm if needed
    if (!m_is_blue && !m_servo_out
        && m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::MANCHE_A_AIR)
    {
        moveArm(OUT);
        m_servo_out = true;
    }

    if (!m_is_first_action && (m_node->now() >= m_moving_timeout_deadline))
    {
        isLate = true;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Robot is late (spent more than "
                        << m_timeout_moving << "seconds trying to reach destination)" << std::endl);
    }

    if (isArrivedAtGoal())
    {
        // retract arm if needed
        if (m_previous_etape_type == Etape::EtapeType::MANCHE_A_AIR)
        {
            moveArm(IN);
        }

        bool l_has_timed_out;
        rclcpp::Time recalageTimeoutDeadline;
        Position position_calage;
        Position l_phare = m_strat_graph->positionCAbsolute(0.2f, 0);
        Position l_coin = m_strat_graph->positionCAbsolute(0.0f, 2.f);
        Position l_position_vitrine = m_strat_graph->positionCAbsolute(0.15f, 0);
        

        float target_angle = 0;
        switch (m_strat_graph->getEtapeEnCours()->getEtapeType())
        {
        case Etape::MOUILLAGE_SUD:
        case Etape::MOUILLAGE_NORD:
            // m_score_match += 20;
            stopAngular();
            stopLinear();

            

            while (true)
            {
                m_strat_mvnt.max_speed.angular.z = 0;
                m_strat_mvnt.max_speed.linear.x = 0;
                publishStratMovement();
                rclcpp::spin_some(m_node);
                usleep(0.1e6);
            }

            break;

        case Etape::STATUETTE:
            stopLinear();
            startAngular();

            target_angle = (l_coin - m_current_pose.getPosition()).getAngle();

            l_has_timed_out = alignWithAngleWithTimeout(Angle(target_angle));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "In front of STATUETTE, orienting towards" << target_angle
                                                                       << std::endl);

            if (l_has_timed_out)
            {
                m_action_aborted = true;
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "orienting toward STATUETTE has timed out :(" << std::endl);
            }
            stopAngular();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Recalage bordure statuette" << std::endl);
            startLinear();
            recalage_bordure();
            publishStratMovement();
            recalageTimeoutDeadline = m_node->now() + rclcpp::Duration(6,0);

            while (m_node->now().seconds() < recalageTimeoutDeadline.seconds())
            {
                rclcpp::spin_some(m_node);
                usleep(0.1e6);
            }

            clamp_mode();
            publishStratMovement();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Grabing STATUETTE" << std::endl);

            m_theThing->grab_statuette();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "STATUETTE caught" << std::endl);

            stopAngular();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Ecartement bordure statuette" << std::endl);
            startLinear();
            recalage_bordure();
            //m_strat_mvnt.reverse_gear = 1;
            override_gear = 1;
            publishStratMovement();
            recalageTimeoutDeadline = m_node->now() + rclcpp::Duration(4,0);

            while (m_node->now().seconds() < recalageTimeoutDeadline.seconds())
            {
                rclcpp::spin_some(m_node);
                usleep(0.1e6);
            }
            override_gear = 2;

            if (m_vacuum_state != STRONG_VACUUM)
            {
                m_action_aborted = true;
                if (m_vacuum_state == OPEN_AIR)
                {
                    m_strat_graph->dropStatuette(); // Count it as dropped, do not attempt to put it
                                                    // in the vitrine
                }
            }

            startAngular();
            startLinear();
            break;

        case Etape::VITRINE:
            stopLinear();

            target_angle = (l_position_vitrine - m_current_pose.getPosition()).getAngle();

            l_has_timed_out = alignWithAngleWithTimeout(Angle(target_angle));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "In front of VITRINE, orienting towards" << target_angle << std::endl);

            if (l_has_timed_out)
            {
                m_action_aborted = true;
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "orienting toward VITRINE has timed out :(" << std::endl);
            }
            stopAngular();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Recalage bordure statuette" << std::endl);
            startLinear();
            recalage_bordure();
            publishStratMovement();
            recalageTimeoutDeadline = m_node->now() + rclcpp::Duration(6,0);

            while (m_node->now().seconds() < recalageTimeoutDeadline.seconds())
            {
                rclcpp::spin_some(m_node);
                usleep(0.1e6);
            }

            clamp_mode();
            publishStratMovement();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Dropping STATUETTE" << std::endl);

            m_theThing->release_statuette();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "STATUETTE dropped" << std::endl);

            stopAngular();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Ecartement bordure vitrine" << std::endl);
            startLinear();
            recalage_bordure();
            //m_strat_mvnt.reverse_gear = 1;
            override_gear = 1;
            publishStratMovement();
            recalageTimeoutDeadline = m_node->now() + rclcpp::Duration(4,0);

            while (m_node->now().seconds() < recalageTimeoutDeadline.seconds())
            {
                rclcpp::spin_some(m_node);
                usleep(0.1e6);
            }
            override_gear = 2;

            startAngular();
            startLinear();
            break;

        case Etape::CARRE_FOUILLE:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "In front of CARRE_FOUILLE, orienting" << std::endl);
            stopLinear();

            if (!m_is_blue)
            {
                target_angle = M_PI;
            }

            l_has_timed_out = alignWithAngleWithTimeout(Angle(target_angle));

            if (l_has_timed_out)
            {
                m_action_aborted = true;
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "orienting toward CARRE_FOUILLE has timed out :(" << std::endl);
            }
            stopAngular();
            clamp_mode();
            publishStratMovement();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Pushing CARRE_FOUILLE" << std::endl);

            pushCarreFouille();

            retractePusher();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "CARRE_FOUILLE done" << std::endl);

            startAngular();
            startLinear();
            break;

        case Etape::EtapeType::MANCHE_A_AIR:

            stopLinear();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "In front of Manche A Air, orienting" << std::endl);

            l_has_timed_out = alignWithAngleWithTimeout(Angle(-M_PI / 2));

            if (l_has_timed_out)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Timeout while orienting");
            }
            
            startLinear();
            recalage_bordure();
            position_calage = m_goal_pose.getPosition();
            position_calage.setY(Distance(position_calage.getY() - Distance(1)));
            m_goal_pose.setPosition(position_calage);
            publishStratMovement();
            recalageTimeoutDeadline = m_node->now() + rclcpp::Duration(6,0);

            while (m_node->now().seconds() < recalageTimeoutDeadline.seconds())
            {
                rclcpp::spin_some(m_node);
                usleep(0.1e6);
            }

            usleep(1.1e6); // Wait for motors to be up again

            if (l_has_timed_out)
            {
                m_action_aborted = true;
            }
            else
            {

                clamp_mode();
            }
            publishStratMovement();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Start Manche A Air" << std::endl);

            if (!m_is_blue)
            {
                moveArm(IN);
            }
            else
            {
                moveArm(OUT);
            }
            usleep(2.5e6);

            //m_strat_mvnt.reverse_gear = 1;
            override_gear = 1;

            startLinear();
            recalage_bordure();
            position_calage.setY(Distance(position_calage.getY() + Distance(1.1)));
            m_goal_pose.setPosition(position_calage);
            recalageTimeoutDeadline = m_node->now() + rclcpp::Duration(2,0);
            publishStratMovement();

            while (m_node->now().seconds() < recalageTimeoutDeadline.seconds())
            {
                rclcpp::spin_some(m_node);
                usleep(0.1e6);
            }
            override_gear = 2;

            startLinear();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Manche A Air Done" << std::endl);
            break;
        case Etape::EtapeType::PHARE:
            stopLinear();

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "In front of Phare, orienting" << std::endl);
            RCLCPP_WARN_STREAM_COND(
              alignWithAngleWithTimeout((l_phare - m_current_pose.getPosition()).getAngle()),
              "Timeout while orienting");

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "MOVING SERVO DOWN" << std::endl);
            stopAngular();
            publishStratMovement();
            moveArm(DOWN);
            usleep(1e6);
            moveArm(DOWN);
            usleep(0.5e6);

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "MOVING SERVO UP" << std::endl);
            moveArm(UP);
            usleep(1.5e6);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Phare Done" << std::endl);
            startAngular();
            break;
        case Etape::EtapeType::BOUEE:
            stopLinear();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Orienting grabber" << std::endl);
            RCLCPP_WARN_STREAM_COND(
              alignWithAngleWithTimeout(
                Angle((m_goal_pose.getPosition() - m_current_pose.getPosition()).getAngle()
                      - m_theThing->getAngle())),
              "Timeout while orienting");
            m_theThing->grab_hexagon(GrabberContent::ANY);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Bouee grabbed" << std::endl);
            break;
        case Etape::EtapeType::PORT:
            m_theThing->release_hexagon_on_ground(GrabberContent::ANY);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Bouee released" << std::endl);
            break;

        case Etape::EtapeType::ASSIETTE:
            m_score_match += m_strat_graph->dropGateau(m_strat_graph->getEtapeEnCours());
            stopLinear();

            /*RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Orienting grabber" << std::endl);
            RCLCPP_WARN_STREAM_COND(
              alignWithAngleWithTimeout(
                Angle((m_goal_pose.getPosition() - m_current_pose.getPosition()).getAngle()
                      - m_claws->getAngle())),
              "Timeout while orienting");*/
            publishStratMovement();
            m_claws->release_pile();

            //recule(rclcpp::Duration(5,0), Distance(0.15));
            startLinear();

            m_claws->setInside();

            reculeDroit(rclcpp::Duration(3,0), Distance(0.14));

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Assiete" << std::endl);
            break;

        case Etape::EtapeType::PILE_GATEAU:
            m_strat_graph->grabGateau(m_strat_graph->getEtapeEnCours());
            stopLinear();

            /*// Ouvre la pince + orientation
            stopLinear();
            m_claws->release_pile();
            //m_strat_mvnt.reverse_gear = 0;
            override_gear = 0;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Orienting grabber" << std::endl);
            RCLCPP_WARN_STREAM_COND(
              alignWithAngleWithTimeout(
                Angle((m_goal_pose.getPosition() - m_current_pose.getPosition()).getAngle()
                      - m_claws->getAngle())),
              "Timeout while orienting");

            m_claws->setInside();

            // Approche
            startLinear(); // est ce que ça suffit à le faire avancer ?
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Grabber approching" << std::endl);
            RCLCPP_WARN_STREAM_COND(isArrivedAtGoal(), "Timeout while advancing");
            m_claws->grab_pile();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Pile Gateau" << std::endl);
            override_gear = 2;*/
            publishStratMovement();
            m_claws->grab_pile();
            m_claws->setInside();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Pile Gateau caught" << std::endl);
            startLinear();

            break;
        default:
            //stopAngular();
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
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Timeout, probable obstacle on the way. Trying another path." << std::endl);
        abortAction();
        goToNextMission();

        if (m_previous_etape_type == Etape::EtapeType::CARRE_FOUILLE)
        {
            pushCarreFouille();
        }
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
    m_first_manche_a_air_done = false;
    m_score_match = 5; // panier present
    m_score_match += 10; // 10 cerises
    m_score_match += 5; // comptage correct

    m_servos_cmd.enable = true;
    /*m_servos_cmd.brak_speed = 10;
    m_servos_cmd.brak_angle = 148;*/
    m_servos_cmd.pavillon_speed = 10;
    m_servos_cmd.pavillon_angle = 50;
    /*m_servos_cmd.s3_speed = 10;
    m_servos_cmd.s3_angle = 148;*/
    moveArm(FOLDED);
    moveArm(UP);

    m_dist_to_goal = 100.;
    m_state_msg_displayed = false;

    /*************************************************
     *                   Main loop                   *
     *************************************************/

    // Initialize time
    m_timeout_moving = 15; // sec
    m_timeout_orient = 10; // sec
    m_is_first_action = true;
    m_servo_out = false;

    m_node.param<bool>("isBlue", m_is_blue, true);

    if (m_is_blue)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Is Blue !" << std::endl);
    }
    else
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Not Blue :'(" << std::endl);
    }

    m_action_aborted = false;

    m_strat_graph = std::make_unique<Coupe2023>(!m_is_blue);

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
    switch (m_state)
    {
    case State::INIT:
        init();
        break;
    case State::WAIT_TIRETTE:
        if (m_state == State::WAIT_TIRETTE && m_tirette && m_remainig_time > rclcpp::Duration(1, 0))
        {
            m_state = State::RUN;
            dropCherries();
        }
        break;
    case State::RUN:
        stateRun();
        break;
    case State::EXIT:
        stateExit();
        break;
    }
    rclcpp::spin_some(m_node);
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
    rclcpp::init(argc, argv, "goalStrat");
    GoalStrat goal_strat;

    rclcpp::Rate r(rclcpp::Duration(0,0.033 * 10e9)); // 100 hz
    int i = 0;
    goal_strat.loop();
    while (rclcpp::ok())
    {
        goal_strat.loop();

        r.sleep();
    }
}
