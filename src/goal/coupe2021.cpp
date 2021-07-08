#include "goal_strategy/coupe2021.h"
#include "goal_strategy/grabber.h"
#include "krabilib/pose.h"
#include "krabilib/strategie/mancheAAir.h"
#include "krabilib/strategie/mouillageNord.h"
#include "krabilib/strategie/mouillageSud.h"
#include "krabilib/strategie/phare.h"
#include "krabilib/strategie/port.h"

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#ifdef QTGUI
#include <QDebug>
#endif

Position Coupe2021::positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left)
{
    return positionC(1.5 - x_yellow_from_top_left, 1 - y_yellow_from_top_left);
}

/**
 * @brief Initialize the graph of goals based on the color
 *
 * @param isYellow
 */
Coupe2021::Coupe2021(const bool isYellow)
  : StrategieV3(isYellow)
{
    setRemainingTime(100 * 1000);
    m_good_mouillage = Etape::EtapeType::DEPART; // No mouillage is good yet

    // Initialisation des tableaux d'étapes
    m_tableau_etapes_total
      = Etape::initTableauEtapeTotal(NOMBRE_ETAPES); // new Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions
    int main_port = Etape::makeEtape(positionC(1.30, 0),
                                     Etape::DEPART); // départ au fond de la zone de départ

    /*int bouee_our_port_1
      = Etape::makeEtape(new Bouee(positionCAbsolute(0.3, 0.4), GrabberContent::RED_CUP));
    int bouee_our_port_2
      = Etape::makeEtape(new Bouee(positionCAbsolute(0.445, 0.515), GrabberContent::GREEN_CUP));
    int bouee_our_port_3
      = Etape::makeEtape(new Bouee(positionCAbsolute(0.445, 1.085), GrabberContent::RED_CUP));
    int bouee_our_port_4
      = Etape::makeEtape(new Bouee(positionCAbsolute(0.3, 1.2), GrabberContent::GREEN_CUP));*/

    int lighthouse = Etape::makeEtape(new Phare(positionC(1.30, 0.8)));
    int out_of_lighthouse = Etape::makeEtape(positionCAbsolute(0.6, 0.3));

    int out_of_main_port = Etape::makeEtape(positionC(0.8, 0));

    Etape::get(main_port)->addVoisins(out_of_main_port);
    /*Etape::get(out_of_main_port)->addVoisins(bouee_our_port_1);
    Etape::get(bouee_our_port_2)->addVoisins(bouee_our_port_1);
    Etape::get(out_of_main_port)->addVoisins(bouee_our_port_3);
    Etape::get(bouee_our_port_4)->addVoisins(bouee_our_port_3);*/

    Etape::get(lighthouse)->addVoisins(out_of_lighthouse);
    Etape::get(out_of_main_port)->addVoisins(out_of_lighthouse);

    int first_air = Etape::makeEtape(new MancheAAir(positionC(1.270, -0.8)));
    int second_air = Etape::makeEtape(new MancheAAir(positionC(0.865, -0.8)));
    int out_of_first_air = Etape::makeEtape(positionC(1.235, -0.6));
    int out_of_second_air = Etape::makeEtape(positionC(0.9, -0.6));

    Etape::get(out_of_second_air)->addVoisins(out_of_first_air);
    Etape::get(first_air)->addVoisins(out_of_first_air);
    Etape::get(second_air)->addVoisins(out_of_second_air);

    m_south_id = Etape::makeEtape(new MouillageSud(positionC(1.250, -0.250)));
    m_north_id = Etape::makeEtape(new MouillageNord(positionC(1.250, 0.680)));
    m_numero_etape_garage = m_south_id;

    Etape::get(m_north_id)->addVoisins(out_of_lighthouse, lighthouse);

    int push_south_bouees = Etape::makeEtape(new Port(positionC(1.150, 0)));
    int push_north_bouees = Etape::makeEtape(new Port(positionC(1.150, 0.4)));
    Etape::get(out_of_lighthouse)->addVoisins(push_north_bouees);

    // Points de passage
    int waypoint_south = Etape::makeEtape(positionC(.860, -0.300));
    int waypoint_out_of_enemy_small_port = Etape::makeEtape(positionC(0.4, -0.4));

    Etape::get(m_south_id)->addVoisins(out_of_second_air, out_of_first_air, waypoint_south);
    Etape::get(out_of_first_air)->addVoisins(waypoint_south);
    Etape::get(out_of_second_air)->addVoisins(waypoint_south, waypoint_out_of_enemy_small_port);
    Etape::get(out_of_main_port)->addVoisins(waypoint_south, waypoint_out_of_enemy_small_port);

    int waypoint_middle_ports = Etape::makeEtape(positionC(0, -0.4));
    int waypoint_out_of_our_small_port = Etape::makeEtape(positionCAbsolute(1.8, 1.5));
    Etape::get(waypoint_out_of_enemy_small_port)->addVoisins(waypoint_middle_ports, waypoint_south);
    Etape::get(waypoint_out_of_our_small_port)->addVoisins(waypoint_middle_ports);

    int waypoint_out_of_push_south_bouees = Etape::makeEtape(positionC(1.020, -0.3));
    Etape::get(waypoint_out_of_push_south_bouees)
      ->addVoisins(push_south_bouees, waypoint_south, out_of_second_air);

    int our_small_port = Etape::makeEtape(positionCAbsolute(1.8, 1.75));
    Etape::get(our_small_port)->addVoisins(waypoint_out_of_our_small_port);

    m_numero_etape_garage = m_south_id; // Must be set!

#ifdef QTGUI
    qDebug() << Etape::getTotalEtapes();
#endif

    m_nombre_etapes = Etape::getTotalEtapes();

    // Lancer Dijkstra
    startDijkstra();
}

Etape::EtapeType Coupe2021::getGoodMouillage()
{
    return m_good_mouillage;
}

/**
 * @brief The the port where we should stop at the end
 *
 * @param a_m_good_mouillage
 */
void Coupe2021::setGoodMouillage(Etape::EtapeType a_good_mouillage)
{
    m_good_mouillage = a_good_mouillage;
    m_numero_etape_garage
      = a_good_mouillage == Etape::EtapeType::MOUILLAGE_SUD ? m_south_id : m_north_id;
    ROS_WARN_STREAM("GOOD MOUILLAGE SET: " << m_good_mouillage << std::endl);
}

/**
 * @brief Convert a EtapeType into a marker
 *
 * @param m output marker
 * @param e input etape
 */
void etape_type_to_marker(visualization_msgs::Marker& m, const Etape::EtapeType& e)
{
    auto& color = m.color;
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.type = visualization_msgs::Marker::CUBE;

    switch (e)
    {
    case Etape::EtapeType::DEPART:
        color.r = 0;
        color.g = 0;
        color.b = 0;
        m.scale.x = 0.01;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        break;
    case Etape::EtapeType::PHARE:
        color.r = 255;
        color.g = 0;
        color.b = 0;
        break;
    case Etape::EtapeType::MANCHE_A_AIR:
        color.r = 255;
        color.g = 255;
        color.b = 0;
        break;
    case Etape::EtapeType::PORT:
        color.r = 0;
        color.g = 255;
        color.b = 255;
        break;
    case Etape::EtapeType::BOUEE:
        color.r = 255;
        color.g = 55;
        color.b = 255;
        break;
    case Etape::EtapeType::MOUILLAGE_NORD:
        color.r = 0;
        color.g = 255;
        color.b = 0;
        break;
    case Etape::EtapeType::MOUILLAGE_SUD:
        color.r = 0;
        color.g = 0;
        color.b = 255;
        break;
    case Etape::EtapeType::POINT_PASSAGE:
        color.r = 0;
        color.g = 0;
        color.b = 255;
        m.scale.x = 0.01;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        m.type = visualization_msgs::Marker::SPHERE;

        break;
    case Etape::EtapeType::ROBOT_VU_ICI:
        color.r = 255;
        color.g = 255;
        color.b = 255;
        m.scale.x = 0.2;
        m.scale.y = 0.2;
        m.scale.z = 0.2;
        break;
    }
    color.a = 1;
}

/**
 * @brief Convert the graph into marker array for debug purpose
 *
 * @param ma marker array
 */
void Coupe2021::debugEtapes(visualization_msgs::MarkerArray& ma)
{

    uint i = 0;
    for (const auto& etape : Etape::getTableauEtapesTotal())
    {
        if (etape)
        {

            // Display etape
            visualization_msgs::Marker m;
            m.header.frame_id = "/map";
            m.header.seq = m_seq++;
            m.ns = "debug_etapes";
            m.id = i++;
            m.action = visualization_msgs::Marker::MODIFY;
            m.pose = Pose(etape->getPosition(), Angle(0));
            etape_type_to_marker(m, etape->getEtapeType());

            if (etape->getNumero() == this->getGoal()->getNumero())
            {
                m.scale.z *= 10;
            }

            m.lifetime = ros::Duration(0); // Does not disapear
            m.frame_locked = true;
            ma.markers.push_back(m);

            // Display lines
            int nb_children = etape->getNbChildren();

            for (int child_id = 0; child_id < nb_children; child_id++)
            {
                // Are we going here?
                Etape* child = etape->getChild(child_id);
                Position l_segment = child->getPosition() - etape->getPosition();
                Position mid_way = Position(
                  Distance((child->getPosition().getX() + etape->getPosition().getX()) / 2),
                  Distance((child->getPosition().getY() + etape->getPosition().getY()) / 2));
                Angle l_direction = l_segment.getAngle();
                double l_distance = l_segment.getNorme();

                visualization_msgs::Marker line;
                line.type = visualization_msgs::Marker::CUBE;
                line.pose = Pose(mid_way, l_direction);
                line.scale.x = l_distance;
                line.scale.y = 0.01;
                line.scale.z = 0.01;
                line.color.r = 255;
                line.color.g = 0;
                line.color.b = 0;
                line.color.a = 0.5;
                line.lifetime = ros::Duration(0); // Does not disapear
                line.frame_locked = true;
                line.action = visualization_msgs::Marker::MODIFY;
                line.ns = "debug_etapes";
                line.header.frame_id = "/map";
                line.header.seq = m_seq++;
                line.id = i++;
                ma.markers.push_back(line);
            }
        }
    }
}

/**
 * @brief Get the score for the current step
 *
 * @param i
 * @return int
 */
int Coupe2021::getScoreEtape(int i)
{
    ROS_WARN_STREAM("getScoreEtape. Time: " << getRemainingTime() << ", GOOD MOUILLAGE = "
                                            << m_good_mouillage << std::endl);
    int l_score = 0;
    switch (this->m_tableau_etapes_total[i]->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::DEPART:
        l_score = 0;
        break;
    case Etape::POINT_PASSAGE:
        l_score = 0;
        break;
    case Etape::PHARE:
        l_score = 200;
        break;
    case Etape::BOUEE:
        l_score = 0;
        break;
    case Etape::MANCHE_A_AIR:
        l_score = 50;
        break;
    case Etape::PORT:
        l_score = 1;
        break;
    case Etape::MOUILLAGE_NORD:
        l_score = 0;
        if (m_good_mouillage == Etape::MOUILLAGE_NORD && getRemainingTime() < 50.f)
        {
            l_score = 200;
        }
        break;
    case Etape::MOUILLAGE_SUD:
        l_score = 0;
        if (m_good_mouillage == Etape::MOUILLAGE_SUD && getRemainingTime() < 50.f)
        {
            l_score = 200;
        }
        break;

    default:
        return 0;
    }
    return l_score;
}
