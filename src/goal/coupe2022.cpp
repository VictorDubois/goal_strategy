#include "goal_strategy/coupe2022.h"
#include "goal_strategy/grabber.h"
#include "krabilib/pose.h"
#include "krabilib/strategie/galerie.h"
#include "krabilib/strategie/statuette.h"
#include "krabilib/strategie/vitrine.h"

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

    statuette_held = false;

    // Initialisation des tableaux d'étapes
    m_tableau_etapes_total
      = Etape::initTableauEtapeTotal(NOMBRE_ETAPES); // new Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions
    int campement = Etape::makeEtape(positionCAbsolute(0.3f, 0.7f),
                                  Etape::DEPART); // départ au fond de la zone de départ
    /*
    int campement = Etape::makeEtape(positionCAbsolute(0.35f, 0.7f),
                                     Etape::POINT_PASSAGE); // départ au fond de la zone de départ
    Etape::get(campement)->addVoisins(depart);                                     
    */
    int out_of_campement = Etape::makeEtape(positionCAbsolute(0.5f, 0.7f), Etape::POINT_PASSAGE);
    Etape::get(campement)->addVoisins(out_of_campement);

    // Carre de fouille
    int fouille_safe = Etape::makeEtape(new CarreFouille(positionCAbsolute(0.8525f, 1.775f)));
    int fouille_mixte_1 = Etape::makeEtape(new CarreFouille(positionCAbsolute(1.2225f, 1.775f)));
    int fouille_mixte_2 = Etape::makeEtape(new CarreFouille(positionCAbsolute(1.4075f, 1.775f)));
    int fouille_mixte_3 = Etape::makeEtape(new CarreFouille(positionCAbsolute(1.5925f, 1.775f)));
    int fouille_mixte_4 = Etape::makeEtape(new CarreFouille(positionCAbsolute(1.7775f, 1.775f)));

    Etape::get(campement)->addVoisins(fouille_safe);
    Etape::get(fouille_safe)->addVoisins(fouille_mixte_1);
    Etape::get(fouille_mixte_1)->addVoisins(fouille_mixte_2);
    Etape::get(fouille_mixte_2)->addVoisins(fouille_mixte_3);
    Etape::get(fouille_mixte_3)->addVoisins(fouille_mixte_4, false);
    Etape::get(campement)->addVoisins(fouille_mixte_4);
    

    Etape::get(out_of_campement)->addVoisins(fouille_safe);

    // Carre de fouille that require to check the resistances
    int fouille_risk_1 = Etape::makeEtape(positionCAbsolute(0.6675f, 1.775f));
    int fouille_risk_2 = Etape::makeEtape(positionCAbsolute(1.0375f, 1.775f));
    Etape::get(campement)->addVoisins(fouille_risk_1);
    Etape::get(fouille_risk_1)->addVoisins(fouille_safe);
    Etape::get(fouille_safe)->addVoisins(fouille_risk_2);
    Etape::get(fouille_risk_2)->addVoisins(fouille_mixte_1);

    // Statuette
    int statuette = Etape::makeEtape(new Statuette(positionCAbsolute(0.45f, 1.55f)));
    Etape::get(campement)->addVoisins(statuette);
    Etape::get(fouille_safe)->addVoisins(statuette);

    int vitrine = Etape::makeEtape(new Vitrine(positionCAbsolute(0.25f, 0.2f)));
    int galerie_bleu = Etape::makeEtape(new Galerie(positionCAbsolute(0.57f, 0.2f)));
    int galerie_vert = Etape::makeEtape(new Galerie(positionCAbsolute(0.81f, 0.2f)));
    int galerie_rouge = Etape::makeEtape(new Galerie(positionCAbsolute(1.05f, 0.2f)));

    Etape::get(out_of_campement)->addVoisins(statuette);
    Etape::get(out_of_campement)->addVoisins(vitrine);
    Etape::get(statuette)->addVoisins(fouille_mixte_1);
    //Etape::get(statuette)->addVoisins(fouille_mixte_4);

    Etape::get(out_of_campement)->addVoisins(galerie_bleu);
    Etape::get(out_of_campement)->addVoisins(galerie_vert);
    Etape::get(out_of_campement)->addVoisins(galerie_rouge);

    Etape::get(galerie_vert)->addVoisins(galerie_bleu);
    Etape::get(galerie_vert)->addVoisins(galerie_rouge);

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
    case Etape::EtapeType::VITRINE:
        color.r = 255;
        color.g = 255;
        color.b = 0;
        break;
    case Etape::EtapeType::STATUETTE:
        color.r = 255;
        color.g = 0;
        color.b = 255;
        break;
    case Etape::EtapeType::GALERIE:
        color.r = 255;
        color.g = 255;
        color.b = 255;
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
    ROS_INFO_STREAM("getScoreEtape. Time: " << getRemainingTime() << std::endl);
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
        l_score = 15;
        break;
    case Etape::VITRINE:
        l_score = 0;
        if (statuette_held)
        {
            l_score = 2;
        }
        break;
    case Etape::STATUETTE:
        l_score = 2;
        break;
    case Etape::GALERIE:
        l_score = 0;
        break;
    default:
        return 0;
    }
    return l_score;
}

void Coupe2022::catchStatuette()
{
    statuette_held = true;
}
void Coupe2022::dropStatuette()
{
    statuette_held = false;
}
