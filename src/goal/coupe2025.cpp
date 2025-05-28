#include "goal_strategy/coupe2025.h"
#include "goal_strategy/grabber.h"
#include "krabilib/pose.h"
#include "krabilib/strategie/aireDeConstruction.h"
#include "krabilib/strategie/plateforme.h"

#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>

#ifdef QTGUI
#include <QDebug>
#endif
//todo delete this when strategie is done and it's not used anymore
Position Coupe2025::positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left)
{
    return positionC(1.5 - x_yellow_from_top_left, 1 - y_yellow_from_top_left);
}
    //  ____________________________________________________
    // |P                       SCENE                        |
    // |A                                                    |
    // |M                                                    |
    // |I                       x                            |
    // |                      <----o               START_SIDE|
    // |                           |                         |
    // |                           |                         |
    // |                           v y                       |  
    // |_______________START_FRONT__________________________ |
    //                         PUBLIC
    //        
    // "us" position are define as blue ("them" are yellow)

/**
 * @brief Initialize the graph of goals based on the color
 *
 * @param isYellow
 * @param starting_position
 */
Coupe2025::Coupe2025(const bool isYellow, const StartingPosition2025 starting_position)
  : StrategieV3(isYellow, true)
{
    setRemainingTime(84 * 1000); // duration of the match

    // Initialisation des tableaux d'étapes
    m_tableau_etapes_total
      = Etape::initTableauEtapeTotal(NOMBRE_ETAPES); // new Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes qui correspondant à des actions sont créées automatiquement lors de l'ajout d'actions
    
    int campement;    

    //Choix du campement
    switch (starting_position)
    { 
    case StartingPosition2025::FRONT_START:
        campement = Etape::makeEtape(positionC(-0.275f, 0.775f),
                                     Etape::DEPART); 
        break;
    case StartingPosition2025::COTE_START:
        campement = Etape::makeEtape(positionC(1.275f, 0.125f),
                                     Etape::DEPART); 
        break;
    case StartingPosition2025::PAMI_START:
        campement = Etape::makeEtape(positionC(-1.125f, -0.775f),
                                     Etape::DEPART); 
        break;
    default:
        throw std::runtime_error("Wrong starting position");
        break;
    }

    //todo crad remove 
    int area_pami_us2, area_pami_us3, area_solar_panel_us2, area_solar_panel_us3;


    
    float distance_to_wall = 0.4f;
    // Definition des zone de stock
    int stock_bord_cote_publique       = Etape::makeEtape(new StockDeMatierePremiere(Pose(positionC(1.425f, 0.6f), Angle(M_PI/2))));
    int stock_bord_cote_publique_them  = Etape::makeEtape(new StockDeMatierePremiere(Pose(positionC(-1.425f, 0.6f), Angle(M_PI/2))));
    int stock_bord_cote_backstage      = Etape::makeEtape(new StockDeMatierePremiere(Pose(positionC(1.425f, -0.325f), Angle(M_PI/2))));
    int stock_bord_cote_backstage_them = Etape::makeEtape(new StockDeMatierePremiere(Pose(positionC(-1.425f, -0.325f), Angle(M_PI/2))));
    int stock_proche_rampe             = Etape::makeEtape(new StockDeMatierePremiere(Pose(positionC(-0.675f, -0.725f), Angle(0.0f))));
    int stock_centre_us                = Etape::makeEtape(new StockDeMatierePremiere(Pose(positionC(-0.4f, 0.05f), Angle(0.0f))));
    int stock_centre_them              = Etape::makeEtape(new StockDeMatierePremiere(Pose(positionC(0.4f, 0.05f), Angle(0.0f))));
    int stock_front_us                 = Etape::makeEtape(new StockDeMatierePremiere(Pose(positionC(-0.725f, 0.75f), Angle(0.0f))));
    int stock_front_them               = Etape::makeEtape(new StockDeMatierePremiere(Pose(positionC(0.725f, 0.75f), Angle(0.0f))));
    
    // Definition de zones de construction
    //Us
    int zone_contruction_front           = Etape::makeEtape(new AireDeConstruction(Pose(positionC(-0.275f, 0.775f), Angle(0.0f)), Owner::us, AireSize::AIRE_BIG));
    int zone_contruction_cote            = Etape::makeEtape(new AireDeConstruction(Pose(positionC(1.275f, 0.125f), Angle(0.0f)), Owner::us, AireSize::AIRE_BIG));
    int zone_contruction_backstage = Etape::makeEtape(new AireDeConstruction(
      Pose(positionC(-1.1f, -0.480f), Angle(0.0f)), 
	Pose(positionC(-1.125f, -0.775f), Angle(0.0f)),
      Owner::us,
      AireSize::AIRE_BIG));
    
    int mini_zone_contruction_coin       = Etape::makeEtape(new AireDeConstruction(Pose(positionC(1.275f, 0.925f), Angle(0.0f)), Owner::us, AireSize::AIRE_SMALL));
    int mini_zone_contruction_front      = Etape::makeEtape(new AireDeConstruction(Pose(positionC(-0.725f, 0.925f), Angle(0.0f)), Owner::us, AireSize::AIRE_SMALL));
    
    //Them
    int zone_contruction_front_them      = Etape::makeEtape(new AireDeConstruction(Pose(positionC(0.275f, 0.775f), Angle(0.0f)), Owner::them, AireSize::AIRE_BIG));
    int zone_contruction_cote_them       = Etape::makeEtape(new AireDeConstruction(Pose(positionC(-1.275f, 0.125f), Angle(0.0f)), Owner::them, AireSize::AIRE_BIG));
    int zone_contruction_them_backstage  = Etape::makeEtape(new AireDeConstruction(Pose(positionC(1.125f, -0.775f), Angle(0.0f)), Owner::them, AireSize::AIRE_BIG));
    

    int mini_zone_contruction_coin_them  = Etape::makeEtape(new AireDeConstruction(Pose(positionC(-1.275f, 0.925f), Angle(0.0f)), Owner::them, AireSize::AIRE_SMALL));
    int mini_zone_contruction_front_them = Etape::makeEtape(new AireDeConstruction(Pose(positionC(0.725f, 0.925f), Angle(0.0f)), Owner::them, AireSize::AIRE_SMALL));

    // Definition des points de passage
    int point_passage_stock_centre_us_publique = Etape::makeEtape(positionC(-0.4f, 0.32f));
    int point_passage_stock_centre_us_back = Etape::makeEtape(positionC(-0.4f, -0.22f));

    int point_passage_stock_centre_them_publique = Etape::makeEtape(positionC(0.4f, 0.32f));
    int point_passage_stock_centre_them_back = Etape::makeEtape(positionC(0.4f, -0.22f));

    int point_passage_zone_contruction_cote = Etape::makeEtape(positionC(1.075f, 0.125f));

    int point_passage_1 = Etape::makeEtape(positionC(0.725f, 0.32f));
    int point_passage_2 = Etape::makeEtape(positionC(0.725f, 0.6f));

    int point_passage_stock_front_us = Etape::makeEtape(positionC(-0.725f, 0.32f));
    int point_passage_3 = Etape::makeEtape(positionC(-0.725f, -0.22f));

    //Definition des liens

    // Simple graph
    Etape::get(zone_contruction_backstage)->addVoisins(point_passage_3);
    Etape::get(point_passage_stock_centre_us_publique)->addVoisins(campement, zone_contruction_front);
    Etape::get(stock_centre_us)->addVoisins(point_passage_stock_centre_us_publique);
    Etape::get(point_passage_stock_front_us)->addVoisins(point_passage_stock_centre_us_publique, stock_front_us);
    Etape::get(stock_front_us)->addVoisins(mini_zone_contruction_front);
    Etape::get(point_passage_3)->addVoisins(point_passage_stock_front_us);
    

    // Hard graph
    /*Etape::get(zone_contruction_backstage)->addVoisins(point_passage_stock_centre_us_back);

    Etape::get(point_passage_stock_centre_us_publique)->addVoisins(campement, zone_contruction_front, point_passage_stock_centre_them_publique);
    Etape::get(stock_centre_us)->addVoisins(point_passage_stock_centre_us_publique, point_passage_stock_centre_us_back);

    Etape::get(stock_centre_them)->addVoisins(point_passage_stock_centre_them_publique, point_passage_stock_centre_them_back);
    
    Etape::get(point_passage_stock_centre_us_back)->addVoisins(point_passage_stock_centre_them_back);

    Etape::get(point_passage_zone_contruction_cote)->addVoisins(point_passage_stock_centre_them_back, point_passage_stock_centre_them_publique, zone_contruction_cote);

    Etape::get(point_passage_stock_centre_them_publique)->addVoisins(point_passage_1);
    Etape::get(point_passage_2)->addVoisins(point_passage_1, stock_bord_cote_publique);    
*/
    m_numero_etape_garage = zone_contruction_backstage; // Must be set!

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
void Coupe2025::etape_type_to_marker(visualization_msgs::msg::Marker& m, Etape* a_etape)
{
    // auto& color = m.color;
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.type = visualization_msgs::msg::Marker::CUBE;
    CoucheGateau type_couche = CoucheGateau::glacage_rose;
    Owner owner;
    AireDeConstruction* l_adc;

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

    case Etape::EtapeType::AIRE_DE_CONSTRUCTION:
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.scale.z = 0.01f;
        m.color.a = 0.5f;
        m.scale.x = 0.45f;
        m.scale.y = 0.45f;
        l_adc = static_cast<AireDeConstruction*>(a_etape->getAction());
        owner = l_adc->getOwner();

        if(!l_adc->isBig())
        {
            m.scale.y = 0.15f;
        }

        // Area blue
        m.color.r = 0.f / 255.f;
        m.color.g = 91.f / 255.f;
        m.color.b = 140.f / 255.f;
        

        if (isYellow() == (owner == Owner::us))
        {
            // Area yellow
            m.color.r = 111.f / 255.f;
            m.color.g = 111.f / 255.f;
            m.color.b = 0.f / 255.f;
        }

        break;
    case Etape::EtapeType::STOCK_MATIERE_PREMIERE:
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.scale.x = 0.4f;
        m.scale.y = 0.1f;
        m.scale.z = 0.01f;
        m.color.r = 222.0f / 255.f;
        m.color.g = 184.f / 255.f;
        m.color.b = 135.0f / 255.f;
        break;

    case Etape::EtapeType::POINT_PASSAGE:
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 1;
        m.scale.x = 0.01;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        m.type = visualization_msgs::msg::Marker::SPHERE;
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
void Coupe2025::debugEtapes(visualization_msgs::msg::MarkerArray& ma)
{
    uint i = 0;
    for (auto& etape : Etape::getTableauEtapesTotal())
    {
        if (etape)
        {

            // Display etape
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            //m.header.seq = m_seq++;
            m.ns = "debug_etapes";
            m.id = i++;
            m.action = visualization_msgs::msg::Marker::MODIFY;
            m.pose = Pose(etape->getPosition(), Angle(0));
            etape_type_to_marker(m, etape);

            if (etape->getEtapeType() == Etape::EtapeType::AIRE_DE_CONSTRUCTION)
            {
                AireDeConstruction* l_adc = static_cast<AireDeConstruction*>(etape->getAction());
                m.pose = l_adc->getAreaCenter();
            }
            else if (etape->getEtapeType() == Etape::EtapeType::STOCK_MATIERE_PREMIERE)
            {
                StockDeMatierePremiere* l_stockMP = static_cast<StockDeMatierePremiere*>(etape->getAction());
                m.pose = l_stockMP->getStockCenter();
            }

            if (etape->getNumero() == this->getGoal()->getNumero())
            {
                m.scale.z *= 10;
                if (etape->getEtapeType() == Etape::EtapeType::AIRE_DE_CONSTRUCTION)
                {
                    m.scale.z = 1;
                }
            }

            m.lifetime = rclcpp::Duration(0, 0); // Does not disapear
            m.frame_locked = true;
            ma.markers.push_back(m);

            visualization_msgs::msg::Marker l_marker_info;
            l_marker_info.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            l_marker_info.pose = Pose(etape->getPosition(), Angle(0));
            l_marker_info.pose.position.z+=0.3f;

            l_marker_info.header.frame_id = "map";
            l_marker_info.ns = "debug_etapes";
            l_marker_info.id = i++;
            l_marker_info.action = visualization_msgs::msg::Marker::ADD;

            l_marker_info.text = etape->getName() + "\n"
		   + std::to_string(etape->getScore()) + "pts\n"
		   + std::to_string(etape->getHeuristicScore()) + "<3\n"
		   + std::to_string(etape->getDistance()) + "mm";

            l_marker_info.scale.x = 0.3;
            l_marker_info.scale.y = 0.3;
            l_marker_info.scale.z = 0.05;

            l_marker_info.color.r = 0.5f;
            l_marker_info.color.g = 0.5f;
            l_marker_info.color.b = 0.2f;
            l_marker_info.color.a = 1.0;

            ma.markers.push_back(l_marker_info);

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

                visualization_msgs::msg::Marker line;
                line.type = visualization_msgs::msg::Marker::CUBE;
                line.pose = Pose(mid_way, l_direction);
                line.scale.x = l_distance;
                line.scale.y = 0.01;
                line.scale.z = 0.01;
                line.color.r = 1;
                line.color.g = 0;
                line.color.b = 0;
                line.color.a = 0.5;
                line.lifetime = rclcpp::Duration(0, 0); // Does not disapear
                line.frame_locked = true;
                line.action = visualization_msgs::msg::Marker::MODIFY;
                line.ns = "debug_etapes";
                line.header.frame_id = "map";
                //line.header.seq = m_seq++;
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
int Coupe2025::getScoreEtape(int i)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "getScoreEtape. Time: " << getRemainingTime() << std::endl);
    int l_score = 0;
    AireDeConstruction* l_area;
    Owner l_owner;
    unsigned int l_stock_area;
    
    switch (this->m_tableau_etapes_total[i]->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::DEPART:
        l_score = 0;
        break;
    case Etape::STOCK_MATIERE_PREMIERE:
        l_score = 3;

        if (m_stock.size() > 0)
        {
            l_score = -3;
        }
        break;
    case Etape::AIRE_DE_CONSTRUCTION:
        l_area = static_cast<AireDeConstruction*>(this->m_tableau_etapes_total[i]->getAction());
        l_owner = l_area->getOwner();
        l_stock_area = l_area->getPlateformes().size();


        if (m_stock.size() && l_owner == Owner::us && l_stock_area == 0)
        {
            l_score = 6;
            if ( l_stock_area <= 6)
            {
                l_score = 12;
            }
            if (l_stock_area == 0)
            {
                l_score = 24;
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

std::vector<Plateforme> Coupe2025::getStock()
{
    return m_stock;
}




void Coupe2025::grabPlateformes(Etape* e)
{
    StockDeMatierePremiere* l_stock;
    std::vector<Plateforme> l_plateformes;

    switch (e->getEtapeType())
    {
    // A faire : remplacer la priorite par le nombre de points obtenables a l'etape
    case Etape::STOCK_MATIERE_PREMIERE:
        l_stock = static_cast<StockDeMatierePremiere*>(e->getAction());
        l_plateformes = l_stock->getPlateformes();
               
        m_stock.insert(m_stock.end(), l_plateformes.begin(), l_plateformes.end());
        break;
        
    default:
        std::cerr << "Not supposed to grab a plateforme from here!" << std::endl;
    }
}

int Coupe2025::dropPlateformes(Etape* e)
{
    int l_scored = 0;
    AireDeConstruction* l_area;
    Plateforme* l_plateforme;
    std::vector<Plateforme> l_stock_area;
    Owner l_owner_area;
    switch (e->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::AIRE_DE_CONSTRUCTION:
        l_area = static_cast<AireDeConstruction*>(e->getAction());
        l_owner_area = l_area->getOwner();

        if (m_stock.size())
        {
          l_area->addPlateforme(m_stock.back());
          m_stock.pop_back();
          //l_scored = 6; //TODO remove when platforms works
        }  
        break;

    default:
        std::cerr << "Not supposed to drop a plateforme there!" << std::endl;
    }

    return l_scored;
}

AireDeConstruction* Coupe2025::getBestAreaForFunny()
{
    for (auto& l_etape : Etape::getTableauEtapesTotal())
    {
        if (l_etape && l_etape->getNumero() == m_numero_etape_garage && l_etape->getEtapeType() == Etape::EtapeType::AIRE_DE_CONSTRUCTION)
        {  
            return static_cast<AireDeConstruction*>(l_etape->getAction());
        }
    }
}