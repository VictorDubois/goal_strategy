#include "goal_strategy/goal.h"
#include <std_msgs/UInt16.h>

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
        m_servos_cmd.s3_angle = 170;
        ROS_DEBUG_STREAM("Actually fold servo" << std::endl);
        break;
    case IN:
        m_servos_cmd.s3_speed = 128;
        m_servos_cmd.s3_angle = 155;
        ROS_DEBUG_STREAM("Actually move servo IN" << std::endl);
        break;
    case OUT:
        m_servos_cmd.s3_speed = 40;
        m_servos_cmd.s3_angle = 23;
        ROS_DEBUG_STREAM("Actually move servo OUT" << std::endl);
        break;
    case UP:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 148;
        ROS_DEBUG_STREAM("Actually move servo UP" << std::endl);
        break;
    case RELEASE:
        m_servos_cmd.s3_angle = 128;
        m_servos_cmd.s3_angle = 128;
        ROS_DEBUG_STREAM("Actually release servo" << std::endl);
        break;
    case DOWN:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 42;
        ROS_DEBUG_STREAM("Actually move servo DOWN" << std::endl);
        break;
    default:
        break;
    }
    m_servos_cmd.enable = true;
    // m_arm_servo_pub.publish(m_servos_cmd);
}

/**
 * @brief 2020/2021 Funny action: Move the servo to raise the flags
 *
 */
void GoalStrat::hissezLesPavillons()
{
    ROS_INFO_STREAM("Hissez les pavillons!" << std::endl);
    m_servos_cmd.pavillon_speed = 128;
    m_servos_cmd.pavillon_angle = 155;
    m_servos_cmd.enable = true;

    if (!m_funny_action_counted)
    {
        m_funny_action_counted = true;
        m_score_match += 10;
    }
    m_arm_servo_pub.publish(m_servos_cmd);
    publishScore();
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
 * @brief Send cmd to disable angular motion
 *
 */
void GoalStrat::stopAngular()
{
    m_strat_mvnt.max_speed.angular.z = 0;
    m_strat_mvnt.orient = 2;
}

/**
 * @brief Send cmd to enable angular motion
 *
 */
void GoalStrat::startAngular()
{
    m_strat_mvnt.max_speed.angular.z = 1;
    m_strat_mvnt.orient = 0;
}

/**
 * @brief Send cmd to disable linear motion
 *
 */
void GoalStrat::stopLinear()
{
    m_strat_mvnt.max_speed.linear.x = 0;
    m_strat_mvnt.orient = 1;
}

/**
 * @brief Send cmd to enable linear motion
 *
 */
void GoalStrat::startLinear()
{
    m_strat_mvnt.max_speed.linear.x = 1;
    m_strat_mvnt.orient = 0;
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
        ROS_DEBUG_STREAM("Etape id: " << etapeId << std::endl);
        ROS_DEBUG_STREAM("goal: " << goal << std::endl);
        ROS_DEBUG_STREAM("type: " << mission_type << std::endl);
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
    ROS_DEBUG_STREAM("orientToAngle " << a_angle << std::endl);
    m_goal_pose.setAngle(a_angle);
    Position goal_position = m_strat_graph->getEtapeEnCours()->getPosition();
    m_goal_pose.setX(goal_position.getX());
    m_goal_pose.setY(goal_position.getY());
    geometry_msgs::PoseStamped l_posestamped;
    l_posestamped.pose = m_goal_pose;
    l_posestamped.header.frame_id = "map";
    m_goal_pose_pub.publish(l_posestamped);

    m_strat_mvnt.goal_pose = l_posestamped;
    m_strat_mvnt.header.frame_id = "map";
    m_strat_mvnt.orient = 1;
    m_strat_movement_pub.publish(m_strat_mvnt);
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
    ROS_DEBUG_STREAM("current pose: x = "
                     << m_current_pose.getPosition().getX()
                     << ", y = " << m_current_pose.getPosition().getY() << ", etape_en_cours: x = "
                     << m_strat_graph->getEtapeEnCours()->getPosition().getX() << ", y = "
                     << m_strat_graph->getEtapeEnCours()->getPosition().getY() << std::endl
                     << "Distance to objective: " << m_dist_to_goal);

    float l_reach_dist = REACH_DIST;
    if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::BOUEE)
    {
        l_reach_dist = theThing.getReach();
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
    ROS_DEBUG_STREAM(m_current_pose.getPosition().getX() << std::endl);

    // Is the tool on the back?
    Angle toolAngle = m_current_pose.getAngle();
    if (m_strat_mvnt.reverse_gear)
    {
        toolAngle += M_PI;
    }

    // Compute angular diff
    Angle angular_error = AngleTools::diffAngle(angle, toolAngle);
    ROS_DEBUG_STREAM("isAlignedWithAngle angle: "
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
    Position goal_position = m_strat_graph->getEtapeEnCours()->getPosition();
    m_goal_pose.setPosition(goal_position);
    geometry_msgs::PoseStamped l_posestamped;

    l_posestamped.pose = m_goal_pose;
    l_posestamped.header.frame_id = "map";
    m_goal_pose_pub.publish(l_posestamped);

    m_strat_mvnt.goal_pose = l_posestamped;
    m_strat_mvnt.header.frame_id = "map";
    m_strat_mvnt.orient = 0;

    m_strat_movement_pub.publish(m_strat_mvnt);
}

/**
 * @brief Update the robot pose and the various transforms
 *
 */
void GoalStrat::updateCurrentPose()
{
    try
    {
        auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
        const auto& transform
          = m_tf_buffer.lookupTransform("map", base_link_id, ros::Time(0)).transform;
        m_current_pose = Pose(transform);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }

    ROS_DEBUG_STREAM("updateCurrentPose: " << m_current_pose << std::endl);
}

/**
 * @brief Perform the transition between two goals
 *
 */
void GoalStrat::goToNextMission()
{
    m_servo_out = false;

    m_moving_timeout_deadline = ros::Time::now() + ros::Duration(m_timeout_moving);

    startLinear();
    m_is_first_action = false;

    m_state_msg_displayed = false;

    m_previous_etape_type = m_strat_graph->getEtapeEnCours()->getEtapeType();

    if (!m_action_aborted)
    {
        ROS_DEBUG_STREAM("etape type: " << m_previous_etape_type
                                        << ", score before: " << m_score_match);
        // Check if we scored points
        switch (m_previous_etape_type)
        {
        case Etape::EtapeType::PHARE:
            m_score_match += 3;
            // Will the phare have enough time to be raised?
            if (m_remainig_time.toSec() > 10.)
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
                ROS_DEBUG_STREAM("Warning, unkown port!" << std::endl);
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
        ROS_DEBUG_STREAM("score after: " << m_score_match << std::endl);
    }

    m_action_aborted = false;

    int strat_graph_status = m_strat_graph->update();

    ROS_INFO_STREAM("New intermediate: " << m_strat_graph->getEtapeEnCours()->getNumero()
                                         << ", which is a "
                                         << m_strat_graph->getEtapeEnCours()->getEtapeType());
    ROS_INFO_STREAM("New objective: " << m_strat_graph->getGoal()->getNumero() << ", which is a "
                                      << m_strat_graph->getGoal()->getEtapeType());

    if (strat_graph_status == -1)
    {
        ROS_DEBUG_STREAM("Graph status is -1: we're done");
        ROS_INFO("All goals accomplished");
        m_state = State::EXIT;
        return;
    }

    return;
}

GoalStrat::GoalStrat()
  : m_tf_listener(m_tf_buffer)
{
    m_goal_init_done = false;
    m_remainig_time = ros::Duration(100, 0);
    m_strat_mvnt.max_speed_at_arrival = 0.1f;
    m_strat_mvnt.orient = 0;
    m_strat_mvnt.max_speed.linear.x = 1.f;
    m_strat_mvnt.max_speed.angular.z = 1.f;
    m_strat_mvnt.reverse_gear = 2; // don't care

    m_goal_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 1000);
    m_arm_servo_pub = m_nh.advertise<krabi_msgs::servos_cmd>("cmd_servos", 1000);
    m_debug_ma_etapes_pub = m_nh.advertise<visualization_msgs::MarkerArray>("debug_etapes", 5);
    m_strat_movement_pub = m_nh.advertise<krabi_msgs::strat_movement>("strat_movement", 5);

    m_remaining_time_match_sub
      = m_nh.subscribe("/remaining_time", 5, &GoalStrat::updateRemainingTime, this);
    m_tirette_sub = m_nh.subscribe("tirette", 5, &GoalStrat::updateTirette, this);
    m_state = State::INIT;
    m_tirette = false;
    std::string actuators_name = "actuators";
    auto l_servo_balloon = std::make_shared<Servomotor>(10, 120);
    auto l_pump_balloon = std::make_shared<Pump>(false);
    auto l_servo_vacuum_left = std::make_shared<Servomotor>(10, 120);
    auto l_servo_vacuum_middle = std::make_shared<Servomotor>(10, 120);
    auto l_servo_vacuum_right = std::make_shared<Servomotor>(10, 120);

    theThing = Grabber(Position(Eigen::Vector2d(0.0, -0.5)), l_servo_balloon, l_pump_balloon);
    m_actuators = Actuators(&m_nh,
                            actuators_name,
                            l_servo_balloon,
                            l_pump_balloon,
                            l_servo_vacuum_left,
                            l_servo_vacuum_middle,
                            l_servo_vacuum_right);
    m_actuators.start();
}

void GoalStrat::publishAll()
{
    while (true)
    {
        usleep(100000);
        updateCurrentPose();
        checkFunnyAction();
        checkStopMatch();
        publishScore();
        m_arm_servo_pub.publish(m_servos_cmd);
        if (m_goal_init_done)
        {
            publishGoal();
            publishDebugInfos();
        }
    }
}

/**
 * @brief Update the tirette state
 *
 * @param tirette msg
 */
void GoalStrat::updateTirette(std_msgs::Bool tirette)
{
    m_tirette = tirette.data;
}

/**
 * @brief Update the remaining time and perform the time sensitive actions like the funny action and
 * stopping the robot at the end of the match
 *
 * @param a_remaining_time_match
 */
void GoalStrat::updateRemainingTime(std_msgs::Duration a_remaining_time_match)
{
    m_remainig_time = a_remaining_time_match.data;
    m_strat_graph->setRemainingTime(m_remainig_time.toSec() * 1000);
    checkFunnyAction();
    checkStopMatch();
}

/**
 * @brief Check if the end of the match arrived and if so stopping the robot
 *
 */
void GoalStrat::checkStopMatch()
{
    const ros::Duration stop_timing = ros::Duration(1.); // 1s before T=0;

    if (m_remainig_time.toSec() < stop_timing.toSec())
    {
        ROS_INFO_STREAM("Match ended, stopping the actuators");
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
void GoalStrat::checkFunnyAction()
{
    const ros::Duration funny_action_timing = ros::Duration(4.); // 4s before T=0;

    if (m_remainig_time.toSec() < funny_action_timing.toSec())
    {
        ROS_INFO_STREAM("Doing the funny action");
        hissezLesPavillons();
    }
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
    ros::Time orientTimeoutDeadline = ros::Time::now() + ros::Duration(m_timeout_orient);

    ros::Rate r(100); // Check at 100Hz the new pose msg
    while (!isAlignedWithAngle(angle) && ros::Time::now().toSec() < orientTimeoutDeadline.toSec())
    {
        alignWithAngle(angle);
        m_strat_mvnt.orient = 1;
        ros::spinOnce();
        r.sleep();
    }
    return ros::Time::now().toSec() >= orientTimeoutDeadline.toSec();
}

/**
 * @brief Choose the best direction for the robot base on the current and future actions
 *
 */
void GoalStrat::chooseGear()
{
    std_msgs::Bool l_reverseGear;
    if (m_previous_etape_type == Etape::EtapeType::MANCHE_A_AIR
        || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PHARE
        || m_previous_etape_type == Etape::EtapeType::PORT)
    {
        l_reverseGear.data = true;
        m_strat_mvnt.reverse_gear = 1;
    }
    else if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::MANCHE_A_AIR
             || m_previous_etape_type == Etape::EtapeType::PHARE
             || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PORT)
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

    ROS_DEBUG_STREAM("currentEtapeType = " << m_strat_graph->getEtapeEnCours()->getEtapeType()
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
    std_msgs::UInt16 l_score_match;
    l_score_match.data = static_cast<uint16_t>(std::ceil(m_score_match));
    // m_score_pub.publish(l_score_match);

    // Hack to use only one arduino for both servos and LCD
    m_servos_cmd.s4_angle = static_cast<uint16_t>(std::ceil(m_score_match));
    m_servos_cmd.s4_speed = static_cast<uint16_t>(std::ceil(m_score_match));
}

/**
 * @brief Publish command to disable actuator
 *
 */
void GoalStrat::stopActuators()
{
    m_servos_cmd.enable = false;
    m_arm_servo_pub.publish(m_servos_cmd);
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
        m_strat_mvnt.max_speed_at_arrival = 0.1f;
    }
    else
    {
        // By default, brake before arriving
        m_strat_mvnt.max_speed_at_arrival = 0.f;
    }
}

/**
 * @brief RUN state of the state machine
 *
 * Here is performed most of the functions
 */
void GoalStrat::stateRun()
{
    updateCurrentPose();

    m_strat_mvnt.max_speed_at_arrival = 0.1f;
    m_strat_mvnt.orient = 0;
    m_strat_mvnt.max_speed.linear.x = 1.f;
    m_strat_mvnt.max_speed.angular.z = 1.f;
    m_strat_mvnt.reverse_gear = 2; // don't care

    bool girouette_south;
    m_nh.param<bool>("isWeathercockSouth", girouette_south, false);

    auto good_mouillage = girouette_south ? Etape::MOUILLAGE_SUD : Etape::MOUILLAGE_NORD;
    m_strat_graph->setGoodMouillage(good_mouillage);

    publishScore();
    publishDebugInfos();
    m_arm_servo_pub.publish(m_servos_cmd);
    bool isLate = false;
    printCurrentAction();

    chooseGear(); // Go in reverse gear if needed
    setMaxSpeedAtArrival();

    // prepare arm if needed
    if (!m_is_blue && !m_servo_out
        && m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::MANCHE_A_AIR)
    {
        moveArm(OUT);
        m_servo_out = true;
    }

    if (!m_is_first_action && (ros::Time::now() >= m_moving_timeout_deadline))
    {
        isLate = true;
        ROS_INFO_STREAM("Robot is late (spent more than "
                        << m_timeout_moving << "seconds trying to reach destination)" << std::endl);
    }

    if (isArrivedAtGoal())
    {
        // retract arm if needed
        if (m_previous_etape_type == Etape::EtapeType::MANCHE_A_AIR)
        {
            moveArm(IN);
        }

        switch (m_strat_graph->getEtapeEnCours()->getEtapeType())
        {
        case Etape::MOUILLAGE_SUD:
        case Etape::MOUILLAGE_NORD:
            m_score_match += 10;
            break;
        case Etape::EtapeType::MANCHE_A_AIR:
            stopLinear();

            ROS_INFO_STREAM("In front of Manche A Air, orienting" << std::endl);
            ROS_WARN_STREAM_COND(alignWithAngleWithTimeout(Angle(-M_PI / 2)),
                                 "Timeout while orienting");
            clamp_mode();

            ROS_INFO_STREAM("Start Manche A Air" << std::endl);

            if (!m_is_blue)
            {
                moveArm(IN);
            }
            else
            {
                moveArm(OUT);
            }
            usleep(2.5e6);

            ROS_INFO_STREAM("Manche A Air Done" << std::endl);
            break;
        case Etape::EtapeType::PHARE:
            stopLinear();

            ROS_INFO_STREAM("In front of Phare, orienting" << std::endl);
            ROS_WARN_STREAM_COND(alignWithAngleWithTimeout(Angle(-M_PI / 2)),
                                 "Timeout while orienting");

            ROS_INFO_STREAM("MOVING SERVO DOWN" << std::endl);
            stopAngular();
            moveArm(DOWN);
            usleep(1e6);
            moveArm(DOWN);
            usleep(0.5e6);

            ROS_INFO_STREAM("MOVING SERVO UP" << std::endl);
            moveArm(UP);
            usleep(1.5e6);
            ROS_INFO_STREAM("Phare Done" << std::endl);
            break;
        case Etape::EtapeType::BOUEE:
            stopLinear();
            ROS_INFO_STREAM("Orienting grabber" << std::endl);
            ROS_WARN_STREAM_COND(
              alignWithAngleWithTimeout(
                Angle((m_goal_pose.getPosition() - m_current_pose.getPosition()).getAngle()
                      - theThing.getAngle())),
              "Timeout while orienting");
            theThing.grab(GrabberContent::ANY);
            ROS_INFO_STREAM("Bouee grabbed" << std::endl);
            break;
        case Etape::EtapeType::PORT:
            theThing.release(GrabberContent::ANY);
            ROS_INFO_STREAM("Bouee released" << std::endl);
            break;
        default:
            ROS_INFO_STREAM("No special action here\n");
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
        ROS_INFO_STREAM("Timeout, probable obstacle on the way. Trying another path." << std::endl);
        abortAction();
        goToNextMission();
    }
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
    ROS_INFO_STREAM("Mission finished, turning off actuators");
}

/**
 * @brief Initialization of the node
 *
 */
void GoalStrat::init()
{
    m_funny_action_counted = false;
    m_first_manche_a_air_done = false;
    m_score_match = 2; // phare posé
    m_servos_cmd.enable = true;
    /*m_servos_cmd.brak_speed = 10;
    m_servos_cmd.brak_angle = 148;*/
    m_servos_cmd.pavillon_speed = 10;
    m_servos_cmd.pavillon_angle = 0;
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

    m_nh.param<bool>("isBlue", m_is_blue, true);

    if (m_is_blue)
    {
        ROS_DEBUG_STREAM("Is Blue !" << std::endl);
    }
    else
    {
        ROS_DEBUG_STREAM("Not Blue :'(" << std::endl);
    }

    m_action_aborted = false;

    m_strat_graph = std::make_unique<Coupe2021>(!m_is_blue);

    m_strat_graph->update();
    m_previous_etape_type = m_strat_graph->getEtapeEnCours()->getEtapeType();
    m_state = State::WAIT_TIRETTE;

    m_running = std::thread(&GoalStrat::publishAll, this);
}

/**
 * @brief State machine that handles the different goals
 *
 * @return int
 */
int GoalStrat::loop()
{
    switch (m_state)
    {
    case State::INIT:
        init();
        break;
    case State::WAIT_TIRETTE:
        if (m_state == State::WAIT_TIRETTE && m_tirette && m_remainig_time > ros::Duration(1, 0))
        {
            m_state = State::RUN;
        }
        break;
    case State::RUN:
        stateRun();
        break;
    case State::EXIT:
        stateExit();
        break;
    }
    ros::spinOnce();
}

/**
 * @brief Publisher to visually debug the strategy graph
 *
 */
void GoalStrat::publishDebugInfos()
{
    visualization_msgs::MarkerArray ma;
    m_strat_graph->debugEtapes(ma);
    m_debug_ma_etapes_pub.publish(ma);
}

// Entry point
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "goalStrat");
    GoalStrat goal_strat;

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        goal_strat.loop();
        r.sleep();
    }
}
