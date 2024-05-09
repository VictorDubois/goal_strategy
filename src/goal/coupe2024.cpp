#include "goal_strategy/coupe2024.h"
#include "goal_strategy/grabber.h"
#include "krabilib/pose.h"
#include "krabilib/strategie/aireDeDepose.h"
#include "krabilib/strategie/galerie.h"
#include "krabilib/strategie/plantGroup.h"
#include "krabilib/strategie/statuette.h"
#include "krabilib/strategie/vitrine.h"

#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>

#ifdef QTGUI
#include <QDebug>
#endif
//todo delete this when strategie is done and it's not used anymore
Position Coupe2024::positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left)
{
    return positionC(1.5 - x_yellow_from_top_left, 1 - y_yellow_from_top_left);
}
    //  ____________________________________________________
    // |                                                 PAMI|
    // |                                                     |
    // |                                                     |
    // |                       x                             |
    // |CENTER                <----o                         |
    // |                           |                         |
    // |                           |                         |
    // |                           v y                       |  
    // |__________________________________________SOLAR_PANEL|
    //                         PUBLIC
    //        
    //

/**
 * @brief Initialize the graph of goals based on the color
 *
 * @param isYellow
 * @param starting_position
 */
Coupe2024::Coupe2024(const bool isYellow, const StartingPosition starting_position)
  : StrategieV3(isYellow, true)
{
    setRemainingTime(90 * 1000);

    // Initialisation des tableaux d'étapes
    m_tableau_etapes_total
      = Etape::initTableauEtapeTotal(NOMBRE_ETAPES); // new Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes qui correspondant à des actions sont créées automatiquement lors de l'ajout d'actions
    
     
    bool test_contournement = false, pami_start = false, center_start = false, solar_panel_start = false;
    int campement;    
    int area_pami_us, area_pami_them, area_center_us, area_center_them, area_solar_panel_us, area_solar_panel_them, 
        group_plant_midi, group_plant_2h, group_plant_4h, group_plant_6h, group_plant_8h, group_plant_10h,
        point_passage_1, point_passage_2, point_passage_3, point_passage_4, point_passage_5, point_passage_6, point_passage_center, point_passage_7;
    

    //todo crad remove 
    int area_pami_us2, area_pami_us3, area_solar_panel_us2, area_solar_panel_us3;


    //Choix du campement
    switch (starting_position)
    {
    case SOLAR_PANEL:
        campement = Etape::makeEtape(positionC(1.275f, 0.775f),
                                     Etape::DEPART); 
        solar_panel_start = true;
        break;
    case CENTER:
        campement = Etape::makeEtape(positionC(-1.275f, 0.0f),
                                     Etape::DEPART); 
        center_start = true;
        break;
    case PAMI:
        campement = Etape::makeEtape(positionC(1.275f, -0.775f),
                                     Etape::DEPART); 
        pami_start = true;
        break;
    default:
        throw std::runtime_error("Wrong starting position");
        break;
    }




    if (test_contournement)
    {
        // int other_side = Etape::makeEtape(
        //   new PlantGroup(positionC(-campement.getPosition().getX(), -campement.getPosition().getY())));
        // Etape::get(campement)->addVoisins(other_side);
    }
    else
    {
        float distance_to_wall = 0.4f;
        area_pami_us
            = Etape::makeEtape(new AireDeDepose(positionC(1.175f, -0.675f), positionC(1.275f, -0.775f), Owner::us, pami_start));

//todo crad remove 
area_pami_us2
            = Etape::makeEtape(new AireDeDepose(positionC(1.175f, -0.675f), positionC(1.275f, -0.775f), Owner::us, pami_start));
area_pami_us3
            = Etape::makeEtape(new AireDeDepose(positionC(1.175f, -0.675f), positionC(1.275f, -0.775f), Owner::us, pami_start));


        area_pami_them
            = Etape::makeEtape(new AireDeDepose(positionC(-1.175f, -0.675f), positionC(-1.275f, -0.775f), Owner::them, false));
        

        area_center_us
            = Etape::makeEtape(new AireDeDepose(positionC(-1.175f, 0.0f), positionC(-1.275f, 0.0f), Owner::us, center_start));
        

        area_center_them
            = Etape::makeEtape(new AireDeDepose( positionC(1.175f, 0.0f), positionC(1.275f, 0.0f), Owner::them, false));


        area_solar_panel_us
            = Etape::makeEtape(new AireDeDepose(positionC(1.175f, 0.675f), positionC(1.275f, 0.775f), Owner::us, solar_panel_start));
//Todo crad remove 
area_solar_panel_us2
            = Etape::makeEtape(new AireDeDepose(positionC(1.175f, 0.675f), positionC(1.275f, 0.775f), Owner::us, solar_panel_start)); 
area_solar_panel_us3
            = Etape::makeEtape(new AireDeDepose(positionC(1.175f, 0.675f), positionC(1.275f, 0.775f), Owner::us, solar_panel_start));

        area_solar_panel_them 
            = Etape::makeEtape(new AireDeDepose( positionC(-1.175f, 0.675f), positionC(-1.275f, 0.775f), Owner::them,false));


        group_plant_midi = Etape::makeEtape(new PlantGroup(positionC(0.0f, -0.5f)));

        group_plant_2h = Etape::makeEtape(new PlantGroup(positionC(-0.5f, -0.3f)));

        group_plant_4h = Etape::makeEtape(new PlantGroup(positionC(-0.5f, 0.3f)));
        
        group_plant_6h = Etape::makeEtape(new PlantGroup(positionC(0.0f, 0.5f)));
        
        group_plant_8h = Etape::makeEtape(new PlantGroup(positionC(0.5f, 0.3f)));
        
        group_plant_10h = Etape::makeEtape(new PlantGroup(positionC(0.5f, -0.3f)));

        point_passage_1 = Etape::makeEtape(positionC(0.8f, 0.6f));
        point_passage_2 = Etape::makeEtape(positionC(0.0f, 0.75f));
        point_passage_3 = Etape::makeEtape(positionC(-0.8f, 0.6f));
        point_passage_4 = Etape::makeEtape(positionC(0.8f, -0.6f));
        point_passage_5 = Etape::makeEtape(positionC(0.0f, -0.75f));
        point_passage_6 = Etape::makeEtape(positionC(-0.8f, -0.6f));
        point_passage_center = Etape::makeEtape(positionC(0.0f, 0.0f));
        point_passage_7 = Etape::makeEtape(positionC((-1.175f-0.5f)/2.0f, -0.15f));



        Etape::get(area_solar_panel_us)->addVoisins(point_passage_1);
        //todo crad remove
        Etape::get(area_solar_panel_us2)->addVoisins(point_passage_1);
        Etape::get(area_solar_panel_us3)->addVoisins(point_passage_1);
        
        
        Etape::get(area_pami_us)->addVoisins(point_passage_4);
        //todo crad remove

        Etape::get(area_pami_us2)->addVoisins(point_passage_4);
        Etape::get(area_pami_us3)->addVoisins(point_passage_4);
        
        
        Etape::get(area_center_us)->addVoisins(point_passage_3);
        Etape::get(area_center_us)->addVoisins(point_passage_6);

        
        Etape::get(point_passage_1)->addVoisins(point_passage_2);
        Etape::get(point_passage_2)->addVoisins(point_passage_3);
        Etape::get(point_passage_4)->addVoisins(point_passage_5);
        Etape::get(point_passage_5)->addVoisins(point_passage_6);



        Etape::get(point_passage_1)->addVoisins(group_plant_10h);
        Etape::get(point_passage_1)->addVoisins(group_plant_8h);
        Etape::get(point_passage_1)->addVoisins(group_plant_6h);

        Etape::get(point_passage_2)->addVoisins(group_plant_6h);
        Etape::get(point_passage_3)->addVoisins(group_plant_4h);
        Etape::get(point_passage_5)->addVoisins(group_plant_midi);
        Etape::get(point_passage_6)->addVoisins(group_plant_2h);



        Etape::get(point_passage_4)->addVoisins(group_plant_8h);
        Etape::get(point_passage_4)->addVoisins(group_plant_10h);
        Etape::get(point_passage_4)->addVoisins(group_plant_midi);


        Etape::get(group_plant_6h)->addVoisins(group_plant_4h);
        Etape::get(group_plant_4h)->addVoisins(group_plant_2h);
        Etape::get(group_plant_2h)->addVoisins(group_plant_midi);
        Etape::get(group_plant_midi)->addVoisins(group_plant_10h);
        Etape::get(group_plant_10h)->addVoisins(group_plant_8h);
        Etape::get(group_plant_8h)->addVoisins(group_plant_6h);

        //Etape::get(campement)->addVoisins(area_solar_panel_us);
        Etape::get(campement)->addVoisins(point_passage_1);
        Etape::get(group_plant_2h)->addVoisins(point_passage_7);
        Etape::get(point_passage_7)->addVoisins(area_center_us);
        //Etape::get(group_plant_4h)->addVoisins(area_center_us);
        //Etape::get(group_plant_10h)->addVoisins(area_pami_us);

        
        Etape::get(point_passage_center)->addVoisins(group_plant_2h);
        Etape::get(point_passage_center)->addVoisins(group_plant_4h);
        Etape::get(point_passage_center)->addVoisins(group_plant_6h);
        Etape::get(point_passage_center)->addVoisins(group_plant_8h);
        Etape::get(point_passage_center)->addVoisins(group_plant_10h);
        Etape::get(point_passage_center)->addVoisins(group_plant_midi);


    }

    m_numero_etape_garage = area_pami_us; // Must be set!

#ifdef QTGUI
    qDebug() << Etape::getTotalEtapes();
#endif

    m_nombre_etapes = Etape::getTotalEtapes();

    // Lancer Dijkstra
    startDijkstra();
}

void Coupe2024::grabPlant(Etape* e)
{
    PlantGroup* l_plant_group;
    std::vector<Plant> l_plants;

    switch (e->getEtapeType())
    {
    // A faire : remplacer la priorite par le nombre de points obtenables a l'etape
    case Etape::PLANT_GROUP:
        l_plant_group = static_cast<PlantGroup*>(e->getAction());
        l_plants = l_plant_group->getPlants();
               
        m_stock.insert(m_stock.end(), l_plants.begin(), l_plants.end());
        
        
        break;
        
    default:
        std::cerr << "Not supposed to grab a gateau from here!" << std::endl;
    }
}

int Coupe2024::dropPlant(Etape* e)
{
    int l_scored = 0;
    AireDeDepose* l_area;
    PlantGroup* l_plant_group;
    std::vector<Plant> l_stock_area;
    Owner l_owner_area;
    switch (e->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::AIRE_DE_DEPOSE:
        l_area = static_cast<AireDeDepose*>(e->getAction());
        l_owner_area = l_area->getOwner();

        if (m_stock.size())
        {
          l_area->addPlants(m_stock);
          m_stock.clear();
          l_scored = 6;
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
void Coupe2024::etape_type_to_marker(visualization_msgs::msg::Marker& m, Etape* a_etape)
{
    // auto& color = m.color;
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.type = visualization_msgs::msg::Marker::CUBE;
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

    case Etape::EtapeType::AIRE_DE_DEPOSE:
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.scale.z = 0.01f;
        m.color.a = 0.5f;
        m.scale.x = 0.45f;
        m.scale.y = 0.45f;
        owner = static_cast<AireDeDepose*>(a_etape->getAction())->getOwner();

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
    case Etape::EtapeType::PLANT_GROUP:
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.scale.x = 0.25f;
        m.scale.y = 0.25f;
        m.scale.z = 0.01f;
        m.color.r = 0.0f / 255.f;
        m.color.g = 240.f / 255.f;
        m.color.b = 0.0f / 255.f;
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
void Coupe2024::debugEtapes(visualization_msgs::msg::MarkerArray& ma)
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

            if (etape->getEtapeType() == Etape::EtapeType::AIRE_DE_DEPOSE)
            {
                m.pose = Pose(static_cast<AireDeDepose*>(etape->getAction())->getAreaCenter(), Angle(0));
            }

            if (etape->getNumero() == this->getGoal()->getNumero())
            {
                m.scale.z *= 10;
                if (etape->getEtapeType() == Etape::EtapeType::AIRE_DE_DEPOSE)
                {
                    m.scale.z = 1;
                }
            }

            m.lifetime = rclcpp::Duration(0, 0); // Does not disapear
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
int Coupe2024::getScoreEtape(int i)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "getScoreEtape. Time: " << getRemainingTime() << std::endl);
    int l_score = 0;
    AireDeDepose* l_area;
    Owner l_owner;
    unsigned int l_stock_area;
    
    switch (this->m_tableau_etapes_total[i]->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::DEPART:
        l_score = 0;
        break;
    case Etape::PLANT_GROUP:
        l_score = 3;

        if (m_stock.size() > 0)
        {
            l_score = -3;
        }
        break;
    case Etape::AIRE_DE_DEPOSE:
        l_area = static_cast<AireDeDepose*>(this->m_tableau_etapes_total[i]->getAction());
        l_owner = l_area->getOwner();
        l_stock_area = l_area->getNumberOfPlants();


        if (m_stock.size() && l_owner == Owner::us && l_stock_area < 18)
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

AireDeDepose* Coupe2024::getBestAreaForFunny()
{
    AireDeDepose* l_farthest_area_to_obstacles = nullptr;
    Distance l_farthest_distance_to_obstacles = Distance(0);
    for (auto& l_etape : Etape::getTableauEtapesTotal())
    {
        if (l_etape && l_etape->getEtapeType() == Etape::EtapeType::AIRE_DE_DEPOSE)
        {  
            auto l_area = static_cast<AireDeDepose*>(l_etape->getAction());
            //RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "DEBUG"<< " Owner is" << l_area->getOwner() << "is starting position"<<l_area->isStartingPosition << std::endl);

            if (l_area->getOwner() != Owner::us || l_area->isStartingPosition())
            {
                // Pas la peine de mettre les roues dans le plat de l'adversaire
                continue;
            }

            auto l_current_distance = Distance(l_etape->getDistanceToPotentialObstacle());
            // Trier par distance aux adversaires
            if (l_current_distance > l_farthest_distance_to_obstacles)
            {
                l_farthest_distance_to_obstacles =  l_current_distance;
                l_farthest_area_to_obstacles = l_area;
            }
        }
    }
    //RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "DEBUG"<< " l_farthest_area_to_obstacles" << l_area->getOwner() << "is starting position"<<l_area->isStartingPosition << std::endl);
    return l_farthest_area_to_obstacles;
}

std::vector<Plant> Coupe2024::getStock()
{
    return m_stock;
}