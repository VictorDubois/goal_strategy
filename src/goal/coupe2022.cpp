#include "goal_strategy/coupe2022.h"
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

Position Coupe2022::positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left)
{
    return positionC(1.5 - x_yellow_from_top_left, 1 - y_yellow_from_top_left);
}

/**
 * @brief Initialize the graph of goals based on the color
 *
 * @param isYellow
 */
Coupe2022::Coupe2022(const bool isYellow)
  : StrategieV3(isYellow)
{
    setRemainingTime(100 * 1000);

    // Initialisation des tableaux d'étapes
    m_tableau_etapes_total
      = Etape::initTableauEtapeTotal(NOMBRE_ETAPES); // new Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions
    int campement = Etape::makeEtape(positionCAbsolute(0.3f, 0.7f),
                                     Etape::DEPART); // départ au fond de la zone de départ

    int fouille_safe = Etape::makeEtape(new CarreFouille(positionCAbsolute(0.8525f, 1.775f)));
    int fouille_mixte_1 = Etape::makeEtape(new CarreFouille(positionCAbsolute(1.2225f, 1.775f)));
    int fouille_mixte_2 = Etape::makeEtape(new CarreFouille(positionCAbsolute(1.4075f, 1.775f)));
    int fouille_mixte_3 = Etape::makeEtape(new CarreFouille(positionCAbsolute(1.5925f, 1.775f)));
    int fouille_mixte_4 = Etape::makeEtape(new CarreFouille(positionCAbsolute(1.7775f, 1.775f)));

    Etape::get(campement)->addVoisins(fouille_safe);
    Etape::get(fouille_safe)->addVoisins(fouille_mixte_1);
    Etape::get(fouille_mixte_1)->addVoisins(fouille_mixte_2);
    Etape::get(fouille_mixte_2)->addVoisins(fouille_mixte_3);
    Etape::get(fouille_mixte_3)->addVoisins(fouille_mixte_4);

    // If we want to check the resistances
    int fouille_risk_1 = Etape::makeEtape(positionCAbsolute(0.6675f, 1.7f));
    int fouille_risk_2 = Etape::makeEtape(positionCAbsolute(1.0375f, 1.7f));
    Etape::get(campement)->addVoisins(fouille_risk_1);
    Etape::get(fouille_risk_1)->addVoisins(fouille_safe);
    Etape::get(fouille_safe)->addVoisins(fouille_risk_2);
    Etape::get(fouille_risk_2)->addVoisins(fouille_mixte_1);

    m_numero_etape_garage = campement; // Must be set!

#ifdef QTGUI
    qDebug() << Etape::getTotalEtapes();
#endif

    m_nombre_etapes = Etape::getTotalEtapes();

    // Lancer Dijkstra
    startDijkstra();
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
    case Etape::EtapeType::CARRE_FOUILLE:
        color.r = 255;
        color.g = 0;
        color.b = 0;
        break;
    /*case Etape::EtapeType::MANCHE_A_AIR:
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
        break;*/
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
void Coupe2022::debugEtapes(visualization_msgs::MarkerArray& ma)
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
int Coupe2022::getScoreEtape(int i)
{
    ROS_WARN_STREAM("getScoreEtape. Time: " << getRemainingTime() << std::endl);
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
    case Etape::CARRE_FOUILLE:
        l_score = 5;
        break;
    default:
        return 0;
    }
    return l_score;
}
