#include "goal_strategy/goal.h"
#include <std_msgs/UInt16.h>

void GoalStrat::moveArm(enum PositionServo position)
{
    switch (position)
    {
    case FOLDED:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 10;
        ROS_DEBUG_STREAM("Actually fold servo" << std::endl);
        break;
    case IN:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 23;
        ROS_DEBUG_STREAM("Actually move servo IN" << std::endl);
        break;
    case OUT:
        m_servos_cmd.brak_speed = 40;
        m_servos_cmd.brak_angle = 156;
        ROS_DEBUG_STREAM("Actually move servo OUT" << std::endl);
        break;
    case UP:
        m_servos_cmd.s3_speed = 128;
        m_servos_cmd.s3_angle = 40;
        ROS_DEBUG_STREAM("Actually move servo UP" << std::endl);
        break;
    case RELEASE:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 128;
        ROS_DEBUG_STREAM("Actually release servo" << std::endl);
        break;
    case DOWN:
        m_servos_cmd.s3_speed = 128;
        m_servos_cmd.s3_angle = 120;
        ROS_DEBUG_STREAM("Actually move servo DOWN" << std::endl);
        break;
    default:
        break;
    }
    m_servos_cmd.enable = true;
    m_arm_servo_pub.publish(m_servos_cmd);
}

void GoalStrat::hissezLesPavillons()
{
    ROS_INFO_STREAM("Hissez les pavillons!" << std::endl);
    m_servos_cmd.pavillon_speed = 128;
    m_servos_cmd.pavillon_angle = 40;
    m_servos_cmd.enable = true;

    if (!m_funny_action_counted)
    {
        m_funny_action_counted = true;
        m_score_match += 10;
    }
    m_arm_servo_pub.publish(m_servos_cmd);
    publishScore();
}

void GoalStrat::stopLinear()
{
    std_msgs::Bool linear;
    linear.data = true;
    m_strat_mvnt.max_speed.linear.x = 0;
    m_strat_mvnt.orient = true;
}

void GoalStrat::startLinear()
{
    std_msgs::Bool linear;
    linear.data = false;
    m_strat_mvnt.max_speed.linear.x = 1;
    m_strat_mvnt.orient = false;
}

bool GoalStrat::isBlue()
{
    return m_is_blue;
}

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

// Entry point
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "goalStrat");
    GoalStrat* goalStrat = new GoalStrat{};

    goalStrat->loop();
}

void GoalStrat::orientToAngle(Angle a_angle)
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
    m_strat_mvnt.orient = true;
    m_strat_movement_pub.publish(m_strat_mvnt);
}

bool GoalStrat::arrivedThere()
{

    m_dist_to_goal
      = (m_current_pose.getPosition() - m_strat_graph->getEtapeEnCours()->getPosition()).getNorme();
    ROS_DEBUG_STREAM("current pose: x = "
                     << m_current_pose.getPosition().getX()
                     << ", y = " << m_current_pose.getPosition().getY() << ", etape_en_cours: x = "
                     << m_strat_graph->getEtapeEnCours()->getPosition().getX() << ", y = "
                     << m_strat_graph->getEtapeEnCours()->getPosition().getY() << std::endl
                     << "Distance to objective: " << m_dist_to_goal);

    if (m_dist_to_goal < REACH_DIST)
    {
        // Make it stop
        stopLinear();

        return true;
    }
    return false;
}

bool GoalStrat::doneOrientingTo(Angle angle)
{
    // Output a goal relative to the robot
    ROS_DEBUG_STREAM(m_current_pose.getPosition().getX() << std::endl);
    orientToAngle(angle);

    // Compute angular diff
    Angle angular_error = AngleTools::diffAngle(angle, m_current_pose.getAngle());
    ROS_DEBUG_STREAM("doneOrientingTo angle: "
                     << angle << " ? current: " << m_current_pose.getAngle()
                     << "angular_error = " << AngleTools::rad2deg(angular_error) << std::endl);

    // When we reached the correct orientation, angularly stop and switch state
    if (abs(angular_error) < REACH_ANG)
    {
        return true;
    }
    return false;
}

void GoalStrat::moveTowardGoal()
{
    Position goal_position = m_strat_graph->getEtapeEnCours()->getPosition();
    m_goal_pose.setX(goal_position.getX());
    m_goal_pose.setY(goal_position.getY());
    geometry_msgs::PoseStamped l_posestamped;

    l_posestamped.pose = m_goal_pose;
    l_posestamped.header.frame_id = "map";
    m_goal_pose_pub.publish(l_posestamped);

    m_strat_mvnt.goal_pose = l_posestamped;
    m_strat_mvnt.header.frame_id = "map";
    m_strat_mvnt.orient = false;

    m_strat_movement_pub.publish(m_strat_mvnt);
}

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
    if (strat_graph_status == -1)
    {
        ROS_DEBUG_STREAM("Graph status is -1: we're done" << std::endl);
        m_state = State::EXIT;
        return;
    }

    return;
}

GoalStrat::GoalStrat()
  : m_tf_listener(m_tf_buffer)
{
    m_funny_action_counted = false;
    m_first_manche_a_air_done = false;
    m_good_mouillage = Etape::EtapeType::DEPART; // disabled for now
    m_score_match = 2;                           // phare posé
    m_servos_cmd.enable = true;
    m_servos_cmd.brak_speed = 10;
    m_servos_cmd.brak_angle = 180;
    m_servos_cmd.pavillon_speed = 10;
    m_servos_cmd.pavillon_angle = 255;

    usleep(1000000);
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
    ros::NodeHandle n;

    m_goal_pose_pub = n.advertise<geometry_msgs::PoseStamped>("goal_pose", 1000);
    m_arm_servo_pub = n.advertise<krabi_msgs::servos_cmd>("cmd_servos", 1000);
    m_debug_ma_etapes_pub = n.advertise<visualization_msgs::MarkerArray>("debug_etapes", 5);
    m_strat_movement_pub = n.advertise<krabi_msgs::strat_movement>("strat_movement", 5);

    // m_score_pub = n.advertise<std_msgs::UInt16>("score", 5);

    m_remaining_time_match_sub
      = n.subscribe("remaining_time", 1000, &GoalStrat::updateRemainingTime, this);
    m_girouette_sub = n.subscribe("girouette_is_south", 1000, &GoalStrat::updateGirouette, this);

    n.param<bool>("isBlue", m_is_blue, true);

    if (m_is_blue)
    {
        ROS_DEBUG_STREAM("Is Blue !" << std::endl);
    }
    else
    {
        ROS_DEBUG_STREAM("Not Blue :'(" << std::endl);
    }

    m_action_aborted = false;

    geometry_msgs::Point point1 = geometry_msgs::Point();
    point1.x = 0;
    point1.y = 0;
    point1.z = 0;
    std::vector<geometry_msgs::Point> etapes;
    etapes.push_back(point1);
    m_strat_graph = new Coupe2019(!isBlue(), etapes);

    m_strat_graph->update();
    m_previous_etape_type = m_strat_graph->getEtapeEnCours()->getEtapeType();
}

void GoalStrat::updateRemainingTime(std_msgs::Duration a_remaining_time_match)
{
    m_remainig_time = a_remaining_time_match.data;
    checkFunnyAction();
    checkStopMatch();
}

void GoalStrat::checkStopMatch()
{
    const ros::Duration stop_timing = ros::Duration(1.); // 1s before T=0;

    if (m_remainig_time.toSec() < stop_timing.toSec())
    {
        ROS_INFO_STREAM("Match ended, stopping the actuators" << std::endl);
        stopActuators();
        m_state = State::EXIT;
        m_strat_mvnt.max_speed.linear.x = 0;
        m_strat_mvnt.max_speed.angular.z = 0;
    }
}

void GoalStrat::checkFunnyAction()
{
    const ros::Duration funny_action_timing = ros::Duration(4.); // 4s before T=0;

    if (m_remainig_time.toSec() < funny_action_timing.toSec())
    {
        ROS_INFO_STREAM("Do the funny action" << std::endl);
        hissezLesPavillons();
    }
}

void GoalStrat::orientToAngleWithTimeout(Angle angleIfBlue, Angle angleIfNotBlue)
{
    Angle angleAction = isBlue() ? angleIfBlue : angleIfNotBlue;
    ros::Time orientTimeoutDeadline = ros::Time::now() + ros::Duration(m_timeout_orient);
    while (!doneOrientingTo(angleAction)
           && ros::Time::now().toSec() < orientTimeoutDeadline.toSec())
    {
        m_strat_mvnt.orient = true;
        usleep(10000);
        ros::spinOnce();
    }
}

void GoalStrat::chooseGear()
{
    std_msgs::Bool l_reverseGear;
    if (m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::MANCHE_A_AIR
        || m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PHARE
        || m_previous_etape_type == Etape::EtapeType::PORT)
    {
        l_reverseGear.data = true;
        m_strat_mvnt.reverse_gear = 1;
    }
    else if (m_previous_etape_type == Etape::EtapeType::MANCHE_A_AIR
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

    ROS_DEBUG_STREAM("######################" << std::endl);
    ROS_DEBUG_STREAM("currentEtapeType = " << m_strat_graph->getEtapeEnCours()->getEtapeType()
                                           << "m_previous_etape_type = " << m_previous_etape_type
                                           << ", reverseGear = " << l_reverseGear.data
                                           << std::endl);
}

void GoalStrat::abortAction()
{
    m_strat_graph->collisionAvoided();
    m_action_aborted = true;
}

void GoalStrat::publishScore()
{
    std_msgs::UInt16 l_score_match;
    l_score_match.data = static_cast<uint16_t>(std::ceil(m_score_match));
    // m_score_pub.publish(l_score_match);

    // Hack to use only one arduino for both servos and LCD
    m_servos_cmd.s4_angle = static_cast<uint16_t>(std::ceil(m_score_match));
    m_servos_cmd.s4_speed = static_cast<uint16_t>(std::ceil(m_score_match));
}

void GoalStrat::updateGirouette(std_msgs::Bool girouette_is_south)
{
    if (girouette_is_south.data)
    {
        m_good_mouillage = Etape::EtapeType::MOUILLAGE_SUD;
    }
    else
    {
        m_good_mouillage = Etape::EtapeType::MOUILLAGE_NORD;
    }
}

void GoalStrat::stopActuators()
{
    m_servos_cmd.enable = false;
    m_arm_servo_pub.publish(m_servos_cmd);
}

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

int GoalStrat::loop()
{
    while (m_state != State::EXIT && ros::ok())
    {
        updateCurrentPose();
        m_strat_graph->setGoodMouillage(m_good_mouillage);
        m_strat_mvnt.max_speed_at_arrival = 0.1f;
        m_strat_mvnt.orient = false;
        m_strat_mvnt.max_speed.linear.x = 1.f;
        m_strat_mvnt.max_speed.angular.z = 1.f;
        m_strat_mvnt.reverse_gear = 2; // don't care

        publishScore();
        publishDebugInfos();
        m_arm_servo_pub.publish(m_servos_cmd);
        bool isLate = false;
        printCurrentAction();

        chooseGear(); // Go in reverse gear if needed
        setMaxSpeedAtArrival();

        // prepare arm if needed
        if (!isBlue() && !m_servo_out
            && m_strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::MANCHE_A_AIR)
        {
            moveArm(OUT);
            m_servo_out = true;
        }

        if (!m_is_first_action && (ros::Time::now() >= m_moving_timeout_deadline))
        {
            isLate = true;
            ROS_INFO_STREAM("Robot is late (spent more than "
                            << m_timeout_moving << "seconds trying to reach destination)"
                            << std::endl);
        }

        if (isLate)
        {
            // If it has been too long
            // Then it means that there is an obstacle on the way
            // The first action is excluded because we wait for the tirette (+ no reason for an
            // opponent + no other way)
            ROS_INFO_STREAM("Timeout, probable obstacle on the way. Trying another path."
                            << std::endl);
            abortAction();
            goToNextMission();
        }
        else if (arrivedThere())
        {
            // retract arm if needed
            if (m_previous_etape_type == Etape::EtapeType::MANCHE_A_AIR)
            {
                moveArm(IN);
            }

            int angleAction = 0;
            switch (m_strat_graph->getEtapeEnCours()->getEtapeType())
            {
            case Etape::MOUILLAGE_SUD:
                if (m_good_mouillage == Etape::MOUILLAGE_SUD && m_remainig_time.toSec() < 10.)
                {
                    // stop !
                    stopLinear();
                    m_score_match += 10;
                    publishScore();
                    while (ros::ok())
                    {
                        ros::spinOnce();
                        usleep(20000);
                    }
                }
                break;
            case Etape::MOUILLAGE_NORD:
                if (m_good_mouillage == Etape::MOUILLAGE_NORD && m_remainig_time.toSec() < 10.)
                {
                    // stop !
                    stopLinear();
                    m_score_match += 10;
                    publishScore();
                    while (ros::ok())
                    {
                        ros::spinOnce();
                        usleep(20000);
                    }
                }
                break;
            case Etape::EtapeType::MANCHE_A_AIR:
                stopLinear();

                if (!isBlue())
                {
                    moveArm(IN);
                }
                else
                {
                    moveArm(OUT);
                }
                usleep(2500000); // 1.5s

                /*ROS_DEBUG_STREAM( "In front of a Manche a Air, orienting" << std::endl);
                orientToAngleWithTimeout(40, 320);

                ROS_DEBUG_STREAM( "MOVING SERVO DOWN" << std::endl);
                moveArm(DOWN);
                usleep(1500000); // 1.5s

                orientToAngleWithTimeout(140, 220);

                ROS_DEBUG_STREAM( "MOVING SERVO UP" << std::endl);
                moveArm(UP);
                usleep(1500000); // 1.5s*/
                ROS_INFO_STREAM("Manche A Air Done" << std::endl);
                break;
            case Etape::EtapeType::PHARE:
                stopLinear();
                ROS_INFO_STREAM("In front of Phare, orienting" << std::endl);
                orientToAngleWithTimeout(Angle(M_PI / 2), Angle(-M_PI / 2));

                ROS_INFO_STREAM("MOVING SERVO DOWN" << std::endl);
                moveArm(DOWN);
                usleep(1000000); // 1.s
                moveArm(DOWN);
                usleep(500000); // .5s

                ROS_INFO_STREAM("MOVING SERVO UP" << std::endl);
                moveArm(UP);
                usleep(1500000); // 1.5s
                ROS_INFO_STREAM("Phare Done" << std::endl);
                break;
            default:
                ROS_INFO_STREAM("No special action here\n");
                break;
            }
            goToNextMission();
        }
        moveTowardGoal();

        usleep(20000);

        ros::spinOnce();
    }

    stopActuators();
    usleep(20000);
    stopActuators();
    usleep(20000);
    stopActuators();
    usleep(20000);

    ROS_INFO_STREAM("Mission accomplished, shutting down" << std::endl);
    return 0;
}

void GoalStrat::publishDebugInfos()
{
    visualization_msgs::MarkerArray ma;
    m_strat_graph->debugEtapes(ma);
    m_debug_ma_etapes_pub.publish(ma);
}
