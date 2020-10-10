#include "Krabi/positionPlusAngle.h"
#include "Krabi/strategie/dijkstra.h"
#include "Krabi/strategie/etape.h"
#include <fstream>
#include <iostream>
//#include "Krabi/goldo2018.h"
#include "goal_strategy/coupe2019.h"
//#include "Krabi/../../stratV3/include/strategie/etape.h"
#include "Krabi/constantes.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "goal_strategy/goal.h"
#include "goal_strategy/servos_cmd.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "tf/transform_datatypes.h"
#include <ros/time.h>

#define SERVO_RIGHT 0
#define SERVO_LEFT 1

#define UP_ANGLE 0.015259
#define DOWN_ANGLE 0.029755
#define RELEASE_ANGLE 0.032043 // 2100

#define LOW_SPEED 0.19607  // 50
#define FAST_SPEED 0.58823 // 150

using namespace goal;

State state = RUN;

void GoalStrat::moveArm(enum PositionServo position)
{
    switch (position)
    {
    case UP:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 50;
        std::cout << "Actually move servo UP" << std::endl;
        break;
    case RELEASE:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 128;
        std::cout << "Actually release servo" << std::endl;
        break;
    case DOWN:
        m_servos_cmd.brak_speed = 128;
        m_servos_cmd.brak_angle = 150;
        std::cout << "Actually move servo DOWN" << std::endl;
        break;
    default:
        break;
    }
    m_servos_cmd.enable = true;
    arm_servo_pub.publish(m_servos_cmd);
}

void GoalStrat::hissezLesPavillons()
{
    std::cout << "Hissez les pavillons!" << std::endl;
    m_servos_cmd.pavillon_speed = 128;
    m_servos_cmd.pavillon_angle = 80;
    m_servos_cmd.enable = true;
    scoreMatch += 10;
    arm_servo_pub.publish(m_servos_cmd);
    publishScore();
}

void stopLinear()
{
    //@TODO stop linear speed
}

void startLinear()
{
    //@TODO stop linear speed
}

bool GoalStrat::isBlue()
{
    return team_color;
}

void GoalStrat::printCurrentAction()
{
    if (!state_msg_displayed)
    {
        int etapeId = strat_graph->getEtapeEnCours()->getNumero();
        Position goal = strat_graph->getEtapeEnCours()->getPosition();
        int mission_type = strat_graph->getEtapeEnCours()->getEtapeType();
        std::cout << "Etape id: " << etapeId << std::endl;
        std::cout << "goal: " << goal.Print() << std::endl;
        std::cout << "type: " << mission_type << std::endl;
        state_msg_displayed = true;
    }
}

// Entry point
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "goalStrat");
    GoalStrat* goalStrat = new GoalStrat{};

    goalStrat->loop();
}

void GoalStrat::orient_to_angle(float a_angle)
{
    std::cout << "Orient_to_angle " << a_angle << std::endl;
    goal_pose.setAngle(a_angle);
    Position goal_position = strat_graph->getEtapeEnCours()->getPosition();
    goal_pose.setX(goal_position.getX());
    goal_pose.setY(goal_position.getY());
    geometry_msgs::PoseStamped l_posestamped;
    l_posestamped.pose = goal_pose.getPose();
    l_posestamped.header.frame_id = "odom";
    goal_pose_pub.publish(l_posestamped);
}

float GoalStrat::compute_angular_diff(float a_angle_1, float a_angle_2)
{
    // Thanks to https://stackoverflow.com/a/7571008
    float phi = fmod(std::abs(a_angle_1 - a_angle_2),
                     360.f); // This is either the distance or 360 - distance
    float distance = phi > 180 ? 360.f - phi : phi;
    return distance;
}

int GoalStrat::arrived_there()
{

    dist_to_goal
      = (currentPosition.getPosition() - strat_graph->getEtapeEnCours()->getPosition()).getNorme();
    std::cout << "current pose: x = " << currentPosition.getPosition().getX()
              << ", y = " << currentPosition.getPosition().getY()
              << ", etape_en_cours: x = " << strat_graph->getEtapeEnCours()->getPosition().getX()
              << ", y = " << strat_graph->getEtapeEnCours()->getPosition().getY() << std::endl;
    printf("Distance to objective: %.2f\n", dist_to_goal);
    fflush(stdout);

    if (dist_to_goal < REACH_DIST)
    {
        // Make it stop
        stopLinear();

        return 1;
    }
    return 0;
}

int GoalStrat::done_orienting_to(float angle)
{
    // Output a goal relative to the robot
    std::cout << currentPosition.getPosition().getX() << std::endl;
    orient_to_angle(angle);

    // Compute angular diff
    unsigned int angular_error = compute_angular_diff(angle, currentPosition.getAngle());
    std::cout << "done_orienting_to angle: " << angle
              << " ? current: " << currentPosition.getAngle()
              << "angular_error = " << angular_error % 360 << std::endl;

    // When we reached the correct orientation, angularly stop and switch state
    if (angular_error < REACH_ANG || angular_error > 360 - REACH_ANG)
    {
        return 1;
    }
    return 0;
}

void GoalStrat::move_toward_goal()
{
    Position goal_position = strat_graph->getEtapeEnCours()->getPosition();
    goal_pose.setX(goal_position.getX());
    goal_pose.setY(goal_position.getY());
    geometry_msgs::PoseStamped l_posestamped;
    l_posestamped.pose = goal_pose.getPose();
    l_posestamped.header.frame_id = "odom";
    goal_pose_pub.publish(l_posestamped);
}

unsigned int GoalStrat::get_angular_diff()
{
    //@TODO return the angular difference between the command and the current state
    return 0;
}

// Returns true if the current action is a baffe (push goldenium/accelerator). Specific to 2019
bool GoalStrat::is_baffe_action()
{
    return strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::ACCELERATOR
           || strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::GOLDENIUM;
}

void GoalStrat::updateCurrentPose(geometry_msgs::Pose newPose)
{
    std::cout << "updateCurrentPose: " << newPose.orientation.z << std::endl;
    currentPosition = PositionPlusAngle(newPose);
}

void GoalStrat::go_to_next_mission()
{
    startLinear();
    isFirstAction = false;
    // Reset timeout
    clock_gettime(CLOCK_MONOTONIC, &begin);

    state_msg_displayed = false;

    previousEtapeType = strat_graph->getEtapeEnCours()->getEtapeType();

    if (!actionaborted)
    {
        std::cout << "etape type: " << previousEtapeType << ", score before: " << scoreMatch;
        // Check if we scored points
        switch (previousEtapeType)
        {
        case Etape::EtapeType::PHARE:
            scoreMatch += 3;
            // Will the phare have enough time to be raised?
            if (remainig_time.toSec() > 10.)
            {
                scoreMatch += 10;
            }
            break;
        case Etape::EtapeType::PORT:
            switch (strat_graph->getEtapeEnCours()->getNumero())
            {
            case 10:
                scoreMatch += 3;
                break;
            case 11:
                scoreMatch += 3;
                break;
            case 17:
                scoreMatch += 6;
                break;
            default:
                std::cout << "Warning, unkown port!" << std::endl;
                scoreMatch += 2;
                break;
            }
            scoreMatch += 30;
            break;
        case Etape::EtapeType::MANCHE_A_AIR:
            // 5 for the first, 10 for the second
            scoreMatch += 5;
            if (firstMancheAAirdone)
            {
                scoreMatch += 5;
            }
            firstMancheAAirdone = true;
            break;
        default:
            break;
        }
        std::cout << "score after: " << scoreMatch << std::endl;
    }

    actionaborted = false;

    int strat_graph_status = strat_graph->update();
    if (strat_graph_status == -1)
    {
        std::cout << "Graph status is -1: we're done" << std::endl;
        state = EXIT;
        return;
    }

    if (is_baffe_action())
    {
        // write_stop_distance_modulation("0.67");
    }
    else
    {
        // write_stop_distance_modulation("1");
    }

    return;
}

GoalStrat::GoalStrat()
{
    firstMancheAAirdone = false;
    m_good_mouillage = Etape::EtapeType::DEPART; // disabled for now
    scoreMatch = 2;                              // phare posé
    m_servos_cmd.enable = true;
    m_servos_cmd.brak_speed = 10;
    m_servos_cmd.brak_angle = 50;
    m_servos_cmd.pavillon_speed = 10;
    m_servos_cmd.pavillon_angle = 255;
    // Initialize stop distance modulation
    // write_stop_distance_modulation("1");

    usleep(1000000);
    displayed_end_msg = false;
    dist_to_goal = 100.;
    state_msg_displayed = false;

    /*************************************************
     *           Variables initialization            *
     *************************************************/

    // Create & attach to SHM segment
    mission_finished = 0;

    /*************************************************
     *                   Main loop                   *
     *************************************************/

    Position pos(200, 1850, true); // strategy.input->color);//1500, isBlue());
    startingPosition = PositionPlusAngle(pos, -M_PI / 2);

    // strat_graph = new Coupe2019(!isBlue(), etapes);
    // while(sendNewMission(strat_graph) != -1) {}
    // strat_graph->update();

    fflush(stdout);

    // Initialize time
    clock_gettime(CLOCK_MONOTONIC, &begin);
    timeoutMoving = 1000; // sec
    timeoutOrient = 500;  // sec
    isFirstAction = true;
    ros::NodeHandle n;
    goal_pose_pub = n.advertise<geometry_msgs::PoseStamped>("goal_pose", 1000);
    arm_servo_pub = n.advertise<goal_strategy::servos_cmd>("cmd_servos", 1000);
    goals_pub = n.advertise<geometry_msgs::PoseArray>("etapes", 5);
    reverse_pub = n.advertise<std_msgs::Bool>("reverseGear", 5);
    stop_linear_pub = n.advertise<std_msgs::Bool>("stopLinearSpeed", 5);
    score_pub = n.advertise<std_msgs::UInt16>("score", 5);

    current_pose_sub = n.subscribe("current_pose", 1000, &GoalStrat::updateCurrentPose, this);
    remaining_time_match_sub
      = n.subscribe("remaining_time", 1000, &GoalStrat::updateRemainingTime, this);

    n.param<bool>("isBlue", team_color, true);

    if (team_color)
    {
        std::cout << "Is Blue !" << std::endl;
    }
    else
    {
        std::cout << "Not Blue :'(" << std::endl;
    }

    actionaborted = false;

    geometry_msgs::Point point1 = geometry_msgs::Point();
    point1.x = 0;
    point1.y = 0;
    point1.z = 0;
    std::vector<geometry_msgs::Point> etapes;
    etapes.push_back(point1);
    strat_graph = new Coupe2019(!isBlue(), etapes);

    strat_graph->update();
    previousEtapeType = strat_graph->getEtapeEnCours()->getEtapeType();
}

void GoalStrat::publishEtapes()
{
    geometry_msgs::PoseArray l_etapes;
    l_etapes.header.frame_id = "odom";
    for (auto etape : strat_graph->getPositions())
    {
        geometry_msgs::Pose l_pose;
        l_pose.position = etape;
        l_etapes.poses.push_back(l_pose);
    }
    goals_pub.publish(l_etapes);
}

void GoalStrat::updateRemainingTime(std_msgs::Duration a_remaining_time_match)
{
    remainig_time = a_remaining_time_match.data;
    checkFunnyAction();
}

void GoalStrat::checkFunnyAction()
{
    const ros::Duration funny_action_timing = ros::Duration(4.); // 4s before T=0;

    if (remainig_time.toSec() < funny_action_timing.toSec())
    {
        std::cout << "Do the funny action" << std::endl;
        hissezLesPavillons();
    }
}

void GoalStrat::orient_to_angle_with_timeout(float angleIfBlue, float angleIfNotBlue)
{
    float angleAction = isBlue() ? angleIfBlue : angleIfNotBlue;
    ros::Time orientTimeoutDeadline = ros::Time::now() + ros::Duration(timeoutOrient);
    while (!done_orienting_to(angleAction)
           && ros::Time::now().toSec() < orientTimeoutDeadline.toSec())
    {
        usleep(10000);
        ros::spinOnce();
    }
}

void GoalStrat::chooseGear()
{
    std_msgs::Bool l_reverseGear;
    if (strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::MANCHE_A_AIR
        || strat_graph->getEtapeEnCours()->getEtapeType() == Etape::EtapeType::PHARE
        || previousEtapeType == Etape::EtapeType::PORT)
    {
        l_reverseGear.data = true;
    }
    else
    {
        l_reverseGear.data = false;
    }

    std::cout << "######################" << std::endl;
    std::cout << "currentEtapeType = " << strat_graph->getEtapeEnCours()->getEtapeType()
              << "previousEtapeType = " << previousEtapeType
              << ", reverseGear = " << l_reverseGear.data << std::endl;
    reverse_pub.publish(l_reverseGear);
}

void GoalStrat::abortAction()
{
    strat_graph->collisionAvoided();
    actionaborted = true;
}

void GoalStrat::publishScore()
{
    std_msgs::UInt16 l_score_match;
    l_score_match.data = static_cast<uint16_t>(std::ceil(scoreMatch));
    score_pub.publish(l_score_match);
}

void GoalStrat::updateGirouette()
{
}

int GoalStrat::loop()
{
    while (state != EXIT && ros::ok())
    {
        strat_graph->setGoodMouillage(m_good_mouillage);
        publishScore();
        arm_servo_pub.publish(m_servos_cmd);
        publishEtapes();
        bool isLate = false;
        printCurrentAction();

        chooseGear(); // Go in reverse gear if needed

        clock_gettime(CLOCK_MONOTONIC, &now);

        if (!isFirstAction && ((now.tv_sec - begin.tv_sec) >= timeoutMoving))
        {
            isLate = true;
            std::cout << "Robot is late (spent more than " << timeoutMoving
                      << "seconds trying to reach destination)" << std::endl;
            fflush(stdout);
        }

        if (isLate && !is_baffe_action())
        {
            // If it has been too long
            // Then it means that there is an obstacle on the way
            // The first action is excluded because we wait for the tirette (+ no reason for an
            // opponent + no other way) The baffe actions are excluded because if something is
            // detected, it must be the wall
            std::cout << "Timeout, probable obstacle on the way. Trying another path." << std::endl;
            abortAction();
            go_to_next_mission();
        }
        else if (arrived_there() || (isLate && is_baffe_action()))
        {
            // if (angle != -1 || done_orienting_to(angle)) {
            int angleAction = 0;
            switch (strat_graph->getEtapeEnCours()->getEtapeType())
            {
            case Etape::MOUILLAGE_SUD:
                if (m_good_mouillage == Etape::MOUILLAGE_SUD && remainig_time.toSec() < 10.)
                {
                    // stop !
                    stopLinear();
                    scoreMatch += 10;
                    publishScore();
                    while (ros::ok())
                    {
                        ros::spinOnce();
                        usleep(20000);
                    }
                }
                break;
            case Etape::MOUILLAGE_NORD:
                if (m_good_mouillage == Etape::MOUILLAGE_NORD && remainig_time.toSec() < 10.)
                {
                    // stop !
                    stopLinear();
                    scoreMatch += 10;
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
                std::cout << "In front of a Manche a Air, orienting" << std::endl;
                orient_to_angle_with_timeout(40, 320);

                std::cout << "MOVING SERVO DOWN" << std::endl;
                moveArm(DOWN);
                usleep(1500000); // 1.5s

                orient_to_angle_with_timeout(140, 220);

                std::cout << "MOVING SERVO UP" << std::endl;
                moveArm(UP);
                usleep(1500000); // 1.5s
                std::cout << "Manche A Air Done" << std::endl;
                break;
            case Etape::EtapeType::PHARE:
                stopLinear();
                std::cout << "In front of Phare, orienting" << std::endl;
                orient_to_angle_with_timeout(90, 270);

                std::cout << "MOVING SERVO DOWN" << std::endl;
                moveArm(DOWN);
                usleep(1500000); // 1.5s

                std::cout << "MOVING SERVO UP" << std::endl;
                moveArm(UP);
                usleep(1500000); // 1.5s
                std::cout << "Phare Done" << std::endl;
                break;
            case Etape::EtapeType::ACCELERATOR:
                // Intentional cascade, they were the same action
            case Etape::EtapeType::GOLDENIUM:
                // Stop moving, not already done if baffing
                stopLinear();

                printf("In front of goldenium/accelerator, orienting\n");
                fflush(stdout);
                angleAction = 320;
                if (isBlue())
                {
                    angleAction = 40;
                }
                clock_gettime(CLOCK_MONOTONIC, &orientTime);
                clock_gettime(CLOCK_MONOTONIC, &now);
                while (!done_orienting_to(angleAction)
                       && ((now.tv_sec - orientTime.tv_sec) < timeoutOrient))
                {
                    clock_gettime(CLOCK_MONOTONIC, &now);
                    usleep(10000);
                    ros::spinOnce();
                }
                printf("MOVING SERVO DOWN\n");
                fflush(stdout);
                moveArm(DOWN);
                usleep(1500000); // 1.5s
                angleAction = 220;
                if (isBlue())
                {
                    angleAction = 140;
                }
                clock_gettime(CLOCK_MONOTONIC, &orientTime);
                clock_gettime(CLOCK_MONOTONIC, &now);
                while (!done_orienting_to(angleAction)
                       && ((now.tv_sec - orientTime.tv_sec) < timeoutOrient))
                {
                    clock_gettime(CLOCK_MONOTONIC, &now);
                    usleep(10000);
                    ros::spinOnce();
                }

                printf("MOVING SERVO UP\n");
                fflush(stdout);
                // Move (both) servos to up position
                moveArm(UP);
                usleep(1500000); // 1.5s
                break;
            default:
                printf("No special action here\n");
                fflush(stdout);
                break;
            }
            go_to_next_mission();
        }
        move_toward_goal();

        usleep(20000);

        ros::spinOnce();
    }

    std::cout << "Mission accomplished, shutting down" << std::endl;
    fflush(stdout);
    return 0;
}
