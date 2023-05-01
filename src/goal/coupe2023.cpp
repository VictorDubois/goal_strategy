#include "goal_strategy/coupe2023.h"
#include "goal_strategy/grabber.h"
#include "krabilib/pose.h"
#include "krabilib/strategie/assiette.h"
#include "krabilib/strategie/galerie.h"
#include "krabilib/strategie/pileGateau.h"
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

Position Coupe2023::positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left)
{
    return positionC(1.5 - x_yellow_from_top_left, 1 - y_yellow_from_top_left);
}

/**
 * @brief Initialize the graph of goals based on the color
 *
 * @param isYellow
 */
Coupe2023::Coupe2023(const bool isYellow)
  : StrategieV3(isYellow, false)
{
    setRemainingTime(100 * 1000);

    // Initialisation des tableaux d'étapes
    m_tableau_etapes_total
      = Etape::initTableauEtapeTotal(NOMBRE_ETAPES); // new Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes correspondant à des actions sont créées automatiquement lors de l'ajout d'actions
    int campement = Etape::makeEtape(positionCAbsolute(0.15f, 0.225f),
                                     Etape::DEPART); // départ au fond de la zone de départ


    bool test_contournement = false;

    if (test_contournement)
    {
        int other_side = Etape::makeEtape(
          new PileGateau(positionCAbsolute(2.7f, 1.675f), CoucheGateau::genoise_marron));
        Etape::get(campement)->addVoisins(other_side);
    }
    else
    {
        float distance_to_wall = 0.4f;
        int assiete_us_0
          = Etape::makeEtape(new Assiette(positionCAbsolute(0.4f, distance_to_wall), positionCAbsolute(0.25f, 0.225f), Owner::us));

        int assiete_them_1
          = Etape::makeEtape(new Assiette(positionCAbsolute(1.5f - 0.375f, 0.25f), Owner::them));

        //Etape::get(assiete_us_0)->addVoisins(assiete_them_1);
        //Etape::get(campement)->addVoisins(assiete_them_1);
        //Etape::get(assiete_us_0)->addVoisins(campement);

        int assiete_us_2
          = Etape::makeEtape(new Assiette(positionCAbsolute(1.5f + 0.375f, distance_to_wall), positionCAbsolute(1.5f + 0.375f, 0.225f), Owner::us));
        //Etape::get(assiete_them_1)->addVoisins(assiete_us_2);

        int assiete_them_3
          = Etape::makeEtape(new Assiette(positionCAbsolute(3.f - 0.225f, 0.25f), Owner::them));
        //Etape::get(assiete_them_3)->addVoisins(assiete_us_2);

        int assiete_us_4
          = Etape::makeEtape(new Assiette(positionCAbsolute(3.f - distance_to_wall, 0.75f), positionCAbsolute(3-0.225f, 0.75f), Owner::us));
        //Etape::get(assiete_them_3)->addVoisins(assiete_us_4);

        int assiete_them_5 = Etape::makeEtape(
          new Assiette(positionCAbsolute(3.f - 0.225f, 2.f - 0.75f), Owner::them));
        // Etape::get(assiete_them_5)->addVoisins(assiete_us_4);// Attention: cerises sur le chemin

        int assiete_us_6
          = Etape::makeEtape(new Assiette(positionCAbsolute(3.f - distance_to_wall, 2.f - distance_to_wall), positionCAbsolute(3-0.225f, 2-0.225f), Owner::us));
        //Etape::get(assiete_them_5)->addVoisins(assiete_us_6);

        int assiete_them_7 = Etape::makeEtape(
          new Assiette(positionCAbsolute(1.5f + 0.375f, 2.f - 0.25f), Owner::them));
        //Etape::get(assiete_them_7)->addVoisins(assiete_us_6);

        int assiete_us_8 = Etape::makeEtape(
          new Assiette(positionCAbsolute(1.5f - 0.375f, 2.f - distance_to_wall), positionCAbsolute(1.5f-0.375f, 2-0.225f), Owner::us));
        //Etape::get(assiete_them_7)->addVoisins(assiete_us_8);

        //////////// Trio départ ////////////
        int pile_glacage_depart = Etape::makeEtape(
          new PileGateau(positionCAbsolute(0.45f + 0.125f, 0.25f), CoucheGateau::glacage_rose));

        int pile_creme_depart = Etape::makeEtape(new PileGateau(
          positionCAbsolute(0.45f + 0.125f + 0.2f, 0.25f), CoucheGateau::creme_jaune));

        int pile_genoise_depart = Etape::makeEtape(
          new PileGateau(positionCAbsolute(1.125f, 0.675f), CoucheGateau::genoise_marron));

        int intermediaire_depart = Etape::makeEtape(positionCAbsolute(0.6f, 0.6f));
        int out_of_depose_cerise = Etape::makeEtape(positionCAbsolute(0.3f, 0.3f));
        Etape::get(campement)->addVoisins(out_of_depose_cerise, false);

        Etape::get(intermediaire_depart)->addVoisins(out_of_depose_cerise, assiete_us_0, pile_glacage_depart, pile_genoise_depart, pile_creme_depart);

        //Etape::get(pile_glacage_depart)->addVoisins(pile_creme_depart);
        //Etape::get(pile_genoise_depart)->addVoisins(pile_creme_depart);

        //Etape::get(pile_glacage_depart)->addVoisins(campement);
        //Etape::get(pile_glacage_depart)->addVoisins(assiete_us_0);

        //Etape::get(assiete_us_0)->addVoisins(pile_glacage_depart);
        //Etape::get(assiete_us_0)->addVoisins(pile_genoise_depart);
        //Etape::get(campement)->addVoisins(pile_glacage_depart);
        //Etape::get(campement)->addVoisins(pile_genoise_depart);


        //////////// Trio symetrique X ////////////
        int pile_glacage_loin = Etape::makeEtape(new PileGateau(
          positionCAbsolute(3.0f - (0.45f + 0.125f), 0.25f), CoucheGateau::glacage_rose));

        int pile_creme_loin = Etape::makeEtape(new PileGateau(
          positionCAbsolute(3.0f - (0.45f + 0.125f + 0.2f), 0.25f), CoucheGateau::creme_jaune));

        int pile_genoise_loin = Etape::makeEtape(
          new PileGateau(positionCAbsolute(3.0f - 1.125f, 0.675f), CoucheGateau::genoise_marron));
        Etape::get(pile_genoise_loin)->addVoisins(pile_genoise_depart);

        //Etape::get(pile_glacage_loin)->addVoisins(pile_creme_loin);
        Etape::get(pile_genoise_loin)->addVoisins(pile_creme_loin);

        //////////// Trio symetrique Y (depart adversaire) ////////////
        int pile_glacage_depart_adv = Etape::makeEtape(
          new PileGateau(positionCAbsolute(0.45f + 0.125f, 2 - 0.25f), CoucheGateau::glacage_rose));

        int pile_creme_depart_adv = Etape::makeEtape(new PileGateau(
          positionCAbsolute(0.45f + 0.125f + 0.2f, 2 - 0.25f), CoucheGateau::creme_jaune));

        int pile_genoise_depart_adv = Etape::makeEtape(
          new PileGateau(positionCAbsolute(1.125f, 2 - 0.675f), CoucheGateau::genoise_marron));

        //Etape::get(pile_glacage_depart_adv)->addVoisins(pile_creme_depart_adv);
        Etape::get(pile_genoise_depart_adv)->addVoisins(pile_creme_depart_adv);

        //Etape::get(pile_glacage_depart)->addVoisins(pile_glacage_depart_adv);

        //////////// Trio symetrique X+Y ////////////
        int pile_glacage_loin_adv = Etape::makeEtape(new PileGateau(
          positionCAbsolute(3 - (0.45f + 0.125f), 2 - 0.25f), CoucheGateau::glacage_rose));

        int pile_creme_loin_adv = Etape::makeEtape(new PileGateau(
          positionCAbsolute(3 - (0.45f + 0.125f + 0.2f), 2 - 0.25f), CoucheGateau::creme_jaune));

        int pile_genoise_loin_adv = Etape::makeEtape(
          new PileGateau(positionCAbsolute(3 - 1.125f, 2 - 0.675f), CoucheGateau::genoise_marron));

        //Etape::get(pile_glacage_loin_adv)->addVoisins(pile_creme_loin_adv);
        Etape::get(pile_genoise_loin_adv)->addVoisins(pile_creme_loin_adv);

        Etape::get(pile_genoise_loin_adv)->addVoisins(pile_genoise_loin, pile_genoise_depart, pile_genoise_depart_adv);
        Etape::get(pile_genoise_depart_adv)->addVoisins(pile_genoise_depart, pile_genoise_loin);

        //Etape::get(assiete_us_4)->addVoisins(pile_genoise_depart);


        int in_front_of_tower = Etape::makeEtape(positionCAbsolute(0.6f, 1.f));
        Etape::get(in_front_of_tower)->addVoisins(pile_genoise_depart, pile_genoise_depart_adv, intermediaire_depart, pile_glacage_depart, pile_creme_depart, pile_creme_depart_adv);

        int in_front_of_tower_loin = Etape::makeEtape(positionCAbsolute(2.4f, 1.f));
        Etape::get(in_front_of_tower_loin)->addVoisins(pile_genoise_loin, pile_genoise_loin_adv, pile_glacage_loin, pile_glacage_loin_adv, pile_creme_loin, pile_creme_loin_adv);


        //Etape::get(assiete_us_2)->addVoisins(pile_genoise_depart);
        Etape::get(assiete_us_2)->addVoisins(pile_genoise_loin);
        //Etape::get(assiete_us_2)->addVoisins(pile_creme_depart);
        //Etape::get(assiete_us_2)->addVoisins(pile_glacage_loin);
        //Etape::get(assiete_us_2)->addVoisins(pile_creme_loin);



        Etape::get(assiete_us_4)->addVoisins(pile_genoise_loin);
        //Etape::get(assiete_us_4)->addVoisins(pile_creme_loin);
        //Etape::get(assiete_us_4)->addVoisins(pile_glacage_loin);
    }

    m_numero_etape_garage = campement; // Must be set!

#ifdef QTGUI
    qDebug() << Etape::getTotalEtapes();
#endif

    m_nombre_etapes = Etape::getTotalEtapes();

    // Lancer Dijkstra
    startDijkstra();
}

void Coupe2023::grabGateau(Etape* e)
{
    Assiette* l_assiette;
    PileGateau* l_pile;
    std::vector<CoucheGateau> l_stock_assiette;
    Owner l_owner;

    switch (e->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::PILE_GATEAU:
        l_pile = static_cast<PileGateau*>(e->getAction());
        m_stock.push_back(l_pile->getTypeCouche());
        break;
    case Etape::ASSIETTE:
        l_assiette = static_cast<Assiette*>(e->getAction());
        l_owner = l_assiette->getOwner();
        l_stock_assiette = l_assiette->getGateaux();

        m_stock.push_back(l_stock_assiette.back());
        l_stock_assiette.pop_back();
        break;

    default:
        std::cerr << "Not supposed to grab a gateau from here!" << std::endl;
    }
}

int Coupe2023::dropGateau(Etape* e)
{
    int l_scored = 0;
    Assiette* l_assiette;
    PileGateau* l_pile;
    std::vector<CoucheGateau> l_stock_assiette;
    Owner l_owner;
    switch (e->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::PILE_GATEAU:
        l_pile = static_cast<PileGateau*>(e->getAction());
        m_stock.push_back(l_pile->getTypeCouche());
        break;
    case Etape::ASSIETTE:
        l_assiette = static_cast<Assiette*>(e->getAction());
        l_owner = l_assiette->getOwner();

        if (m_stock.size())
        {
          l_assiette->addGateau(m_stock.back());
          m_stock.pop_back();
          l_scored = 3;
        }  
        break;

    default:
        std::cerr << "Not supposed to drop a gateau there!" << std::endl;
    }

    return l_scored;
}

/**
 * @brief Convert a EtapeType into a marker
 *
 * @param m output marker
 * @param e input etape
 */
void Coupe2023::etape_type_to_marker(visualization_msgs::Marker& m, Etape* a_etape)
{
    // auto& color = m.color;
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.type = visualization_msgs::Marker::CUBE;
    CoucheGateau type_couche = CoucheGateau::glacage_rose;
    Owner owner;

    const Etape::EtapeType& e = a_etape->getEtapeType();
    m.color.a = 1;

    switch (e)
    {
    case Etape::EtapeType::DEPART:
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 0;
        m.scale.x = 0.01;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        break;

    case Etape::EtapeType::ASSIETTE:
        m.type = visualization_msgs::Marker::CUBE;
        m.scale.z = 0.01f;
        m.color.a = 0.5f;
        m.scale.x = 0.45f;
        m.scale.y = 0.45f;
        owner = static_cast<Assiette*>(a_etape->getAction())->getOwner();

        // Assiete bleue
        m.color.r = 0.f / 255.f;
        m.color.g = 91.f / 255.f;
        m.color.b = 140.f / 255.f;
        if (isYellow() == (owner == Owner::us))
        {
            // Assiete verte
            m.color.r = 0.f / 255.f;
            m.color.g = 111.f / 255.f;
            m.color.b = 61.f / 255.f;
        }

        break;
    case Etape::EtapeType::PILE_GATEAU:
        m.type = visualization_msgs::Marker::CYLINDER;
        m.scale.x = 0.12;
        m.scale.y = 0.12;
        type_couche = static_cast<PileGateau*>(a_etape->getAction())->getTypeCouche();
        switch (type_couche)
        {
        case CoucheGateau::genoise_marron:
            m.color.r = 76.f / 255.f;
            m.color.g = 43.f / 255.f;
            m.color.b = 32.f / 255.f;
            break;
        case CoucheGateau::creme_jaune:
            m.color.r = 247.f / 255.f;
            m.color.g = 181.f / 255.f;
            m.color.b = 0;
            break;
        case CoucheGateau::glacage_rose:
            m.color.r = 188.f / 255.f;
            m.color.g = 64.f / 255.f;
            m.color.b = 119.f / 255.f;
            break;
        default:
            m.color.r = 0;
            m.color.g = 1;
            m.color.b = 0;
            break;
        }

        break;
    case Etape::EtapeType::POINT_PASSAGE:
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 1;
        m.scale.x = 0.01;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        m.type = visualization_msgs::Marker::SPHERE;

        break;
    case Etape::EtapeType::ROBOT_VU_ICI:
        m.color.r = 1;
        m.color.g = 1;
        m.color.b = 1;
        m.scale.x = 0.2;
        m.scale.y = 0.2;
        m.scale.z = 0.2;
        break;
    }
}

/**
 * @brief Convert the graph into marker array for debug purpose
 *
 * @param ma marker array
 */
void Coupe2023::debugEtapes(visualization_msgs::MarkerArray& ma)
{
    uint i = 0;
    for (auto& etape : Etape::getTableauEtapesTotal())
    {
        if (etape)
        {

            // Display etape
            visualization_msgs::Marker m;
            m.header.frame_id = "map";
            m.header.seq = m_seq++;
            m.ns = "debug_etapes";
            m.id = i++;
            m.action = visualization_msgs::Marker::MODIFY;
            m.pose = Pose(etape->getPosition(), Angle(0));
            etape_type_to_marker(m, etape);

            if (etape->getEtapeType() == Etape::EtapeType::ASSIETTE)
            {
                m.pose = Pose(static_cast<Assiette*>(etape->getAction())->getAssietteCenter(), Angle(0));
            }

            if (etape->getNumero() == this->getGoal()->getNumero())
            {
                m.scale.z *= 10;
                if (etape->getEtapeType() == Etape::EtapeType::ASSIETTE)
                {
                    m.scale.z = 1;
                }
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
                line.color.r = 1;
                line.color.g = 0;
                line.color.b = 0;
                line.color.a = 0.5;
                line.lifetime = ros::Duration(0); // Does not disapear
                line.frame_locked = true;
                line.action = visualization_msgs::Marker::MODIFY;
                line.ns = "debug_etapes";
                line.header.frame_id = "map";
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
int Coupe2023::getScoreEtape(int i)
{
    ROS_INFO_STREAM("getScoreEtape. Time: " << getRemainingTime() << std::endl);
    int l_score = 0;
    Assiette* l_assiette;
    Owner l_owner;
    unsigned int l_stock_assiette;
    switch (this->m_tableau_etapes_total[i]->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::DEPART:
        l_score = 0;
        break;
    case Etape::PILE_GATEAU:
        l_score = 3;

        if (m_stock.size() > 0)
        {
            l_score = -3;
        }
        break;
    case Etape::ASSIETTE:
        l_assiette = static_cast<Assiette*>(this->m_tableau_etapes_total[i]->getAction());
        l_owner = l_assiette->getOwner();
        l_stock_assiette = l_assiette->getNumberOFGateaux();


        if (m_stock.size() && l_owner == Owner::us && l_stock_assiette < 3)
        {
            l_score = 3;
            if (m_stock.size() >= 2 && l_stock_assiette <= 1)
            {
                l_score = 6;
            }
            if (m_stock.size() >= 4 && l_stock_assiette == 0)
            {
                l_score = 9;
            }
        }
        break;
    case Etape::POINT_PASSAGE:
        l_score = 0;
        break;
    default:
        return 0;
    }
    return l_score;
}
