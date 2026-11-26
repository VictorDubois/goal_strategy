#include "goal_strategy/coupe2027.h"
#include "krabilib/pose.h"

#include <cmath>
#include <iostream>
#include <tf2_ros/transform_listener.h>

// todo delete this when strategie is done and it's not used anymore
Position Coupe2027::positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left)
{
    return positionC(1.5 - x_yellow_from_top_left, 1 - y_yellow_from_top_left);
}
//  ____________________________________________________
// |                                                     |
// |                                                     |
// |                                                     |
// |                        x                            |
// |                      <----o              OUR CHATEAU|
// |                           |                         |
// |                           |                         |
// |                           v y                       |
// |____________________________________________________ |
//                         PUBLIC
//
// "us" position are define as blue ("them" are yellow)

/**
 * @brief Initialize the graph of goals based on the color
 *
 * All the coordinates below are placeholders (the figures of the rules are not in the text).
 *
 * @param isYellow
 */
Coupe2027::Coupe2027(const bool isYellow, rclcpp::Node::SharedPtr a_node)
  : StrategieV3(isYellow, true)
  , m_node(a_node)
{
    setRemainingTime(DUREE_MATCH_MS);

    m_stock = StockPierres();

    m_tableau_etapes_total = Etape::initTableauEtapeTotal(NOMBRE_ETAPES);

    // The base (salle du trone) is also the starting point
    int base = Etape::makeEtape(positionC(-1.25f, -0.0f), "BASE", Etape::DEPART);

    // TODO: orientation of the robot (angle) depends on the actuator used
    float reach = 0.22f; // Distance between the actuator and the center of the wheels, in meters

    int carriere_us_1
      = Etape::makeEtape(new CarrierePierres(Pose(positionC(-1.0f + reach, -0.4f), Angle(0)),
                                             Pose(positionC(-1.0f, -0.4f), Angle(0))),
                         "carriere_us_1");

    int emplacement_mur_us_1
      = Etape::makeEtape(new EmplacementMur(Pose(positionC(-1.0f + reach, -0.75f), Angle(0)),
                                            Pose(positionC(-1.0f, -0.75f), Angle(0))),
                         "emplacement_mur_us_1");

    int emplacement_tour_us_1
      = Etape::makeEtape(new EmplacementTour(Pose(positionC(-0.5f + reach, -0.75f), Angle(0)),
                                             Pose(positionC(-0.5f, -0.75f), Angle(0))),
                         "emplacement_tour_us_1");

    // The Graal is picked up in the base and carried on top of the robot until the end
    int graal = Etape::makeEtape(new Graal(Pose(positionC(-1.35f, 0.1f), Angle(0))), "graal");

    int point_passage_centre = Etape::makeEtape(positionC(-0.7f, -0.2f), Etape::POINT_PASSAGE);

    Etape::get(base)->addVoisins(graal, emplacement_mur_us_1, point_passage_centre);
    Etape::get(graal)->addVoisin(base);
    Etape::get(emplacement_mur_us_1)->addVoisins(base, point_passage_centre);
    Etape::get(carriere_us_1)->addVoisins(point_passage_centre, emplacement_tour_us_1);
    Etape::get(emplacement_tour_us_1)->addVoisins(carriere_us_1, point_passage_centre);
    Etape::get(point_passage_centre)
      ->addVoisins(base, emplacement_mur_us_1, carriere_us_1, emplacement_tour_us_1);

    m_numero_etape_garage = base; // Must be set!

    m_nombre_etapes = Etape::getTotalEtapes();

    // Lancer Dijkstra
    startDijkstra();
}

bool Coupe2027::isTempsRetourBase()
{
    // getRemainingTime() is in seconds and counts down from the duration of the match
    return getRemainingTime() * 1000 < (DUREE_MATCH_MS - TEMPS_RETOUR_BASE_MS);
}

/**
 * @brief Convert a EtapeType into a marker
 *
 * @param m output marker
 * @param e input etape
 */
void Coupe2027::etape_type_to_marker(visualization_msgs::msg::Marker& m, Etape* a_etape)
{
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.color.a = 1;

    switch (a_etape->getEtapeType())
    {
    case Etape::EtapeType::DEPART:
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 0;
        m.scale.x = 0.01;
        m.scale.y = 0.01;
        m.scale.z = 0.01;
        break;

    case Etape::EtapeType::CARRIERE_PIERRES:
        m.scale.x = 0.32f; // size of a pierre
        m.scale.y = 0.11f;
        m.scale.z = 0.03f;
        m.color.a = 0.5f;
        m.color.r = 0.6f;
        m.color.g = 0.4f;
        m.color.b = 0.2f;
        break;

    case Etape::EtapeType::EMPLACEMENT_MUR:
        m.scale.x = 0.2f; // zone of a mur
        m.scale.y = 0.4f;
        m.scale.z = 0.01f;
        m.color.r = 0.8f;
        m.color.g = 0.8f;
        m.color.b = 0.8f;
        break;

    case Etape::EtapeType::EMPLACEMENT_TOUR:
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.scale.x = 0.2f; // diameter of a tour
        m.scale.y = 0.2f;
        m.scale.z = 0.01f;
        m.color.r = 0.5f;
        m.color.g = 0.5f;
        m.color.b = 0.5f;
        break;

    case Etape::EtapeType::GRAAL:
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.scale.x = 0.06f;
        m.scale.y = 0.06f;
        m.scale.z = 0.06f;
        m.color.r = 1.0f;
        m.color.g = 0.8f;
        m.color.b = 0.0f;
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
    default:
        break;
    }

    if (a_etape->aEviter())
    {
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 0;
        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;
        return;
    }
}

/**
 * @brief Convert the graph into marker array for debug purpose
 *
 * @param ma marker array
 */
void Coupe2027::debugEtapes(visualization_msgs::msg::MarkerArray& ma)
{
    uint i = 0;
    for (auto& etape : Etape::getTableauEtapesTotal())
    {
        if (etape)
        {

            // Display etape
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            // m.header.seq = m_seq++;
            m.ns = "debug_etapes";
            m.id = i++;
            m.action = visualization_msgs::msg::Marker::MODIFY;
            m.pose = Pose(etape->getPosition(), Angle(0));
            etape_type_to_marker(m, etape);

            if (etape->getEtapeType() == Etape::EtapeType::CARRIERE_PIERRES)
            {
                m.pose = static_cast<CarrierePierres*>(etape->getAction())->getAreaCenter();
            }
            else if (etape->getEtapeType() == Etape::EtapeType::EMPLACEMENT_MUR)
            {
                m.pose = static_cast<EmplacementMur*>(etape->getAction())->getAreaCenter();
            }
            else if (etape->getEtapeType() == Etape::EtapeType::EMPLACEMENT_TOUR)
            {
                m.pose = static_cast<EmplacementTour*>(etape->getAction())->getAreaCenter();
            }
            else if (etape->getEtapeType() == Etape::EtapeType::GRAAL)
            {
                m.pose = static_cast<Graal*>(etape->getAction())->getAreaCenter();
            }

            if (etape->getNumero() == this->getGoal()->getNumero())
            {
                m.scale.z *= 10;
            }

            m.lifetime = rclcpp::Duration(0, 0); // Does not disapear
            m.frame_locked = true;
            ma.markers.push_back(m);

            visualization_msgs::msg::Marker l_marker_info;
            l_marker_info.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            l_marker_info.pose = Pose(etape->getPosition(), Angle(0));
            l_marker_info.pose.position.z += 0.3f;

            l_marker_info.header.frame_id = "map";
            l_marker_info.ns = "debug_etapes";
            l_marker_info.id = i++;
            l_marker_info.action = visualization_msgs::msg::Marker::ADD;

            l_marker_info.text = etape->getName() + "\n" + std::to_string(etape->getScore())
                                 + "pts\n" + std::to_string(etape->getHeuristicScore()) + "<3\n"
                                 + std::to_string(etape->getDistance()) + "mm\n"
              /*+
etape->getCustomName()*/
              ;

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
                Etape* child = etape->getChild(child_id);

                // Is the line bidirectional?
                bool is_bidirectional = false;
                auto child_children = child->getChildren();
                for (int i = 0; i < child->getNbChildren(); i++)
                {
                    if (child_children[i]->getNumero() == etape->getNumero())
                    {
                        is_bidirectional = true;
                        break;
                    }
                }

                bool is_on_the_path = false;
                auto l_etape_path = this->getGoal();
                while (true)
                {
                    auto l_etape_parent = l_etape_path->getParent();

                    // Is this link on the path to the goal?
                    if ((l_etape_path->getNumero() == etape->getNumero()
                         && l_etape_parent->getNumero() == child->getNumero())
                        || (l_etape_path->getNumero() == child->getNumero()
                            && l_etape_parent->getNumero() == etape->getNumero()))
                    {
                        is_on_the_path = true;
                    }

                    if (l_etape_parent->getDistance() <= 0)
                    {
                        // We have arrived to the current pose, no need to update further
                        break;
                    }

                    l_etape_path = l_etape_parent;
                }

                // Are we going here?
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

                if (is_on_the_path)
                {
                    line.scale.y *= 4;
                    line.scale.z *= 4;
                }
                line.color.r = 1;
                line.color.g = 0;
                line.color.b = 0;
                line.color.a = 0.5;

                if (!is_bidirectional)
                {
                    line.color.a = 0.4;
                    line.type = visualization_msgs::msg::Marker::ARROW;
                    line.pose = Pose(etape->getPosition(), l_direction);
                }

                line.lifetime = rclcpp::Duration(0, 0); // Does not disapear
                line.frame_locked = true;
                line.action = visualization_msgs::msg::Marker::MODIFY;
                line.ns = "debug_etapes";
                line.header.frame_id = "map";
                // line.header.seq = m_seq++;
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
int Coupe2027::getScoreEtape(int i)
{
    int l_score = 0;
    Etape* l_etape = this->m_tableau_etapes_total[i];

    // The opponent PAMIs can attack after TEMPS_RETOUR_BASE_MS: stay in the base (only the
    // actions inside the base remain).
    if (isTempsRetourBase())
    {
        // TODO: only allow the etapes inside the base (construction, Graal)
        return l_etape->getEtapeType() == Etape::DEPART ? 10 : 0;
    }

    switch (l_etape->getEtapeType())
    {
    case Etape::DEPART:
        l_score = 0;
        break;
    case Etape::CARRIERE_PIERRES:
        // 1 point for each pierre placed, only if we have room to carry them
        l_score = m_stock.peutRamasser() ? 3 : 0;
        break;
    case Etape::EMPLACEMENT_MUR:
        if (static_cast<EmplacementMur*>(l_etape->getAction())->isPorte())
        {
            l_score = m_stock.peutConstruirePorte() ? 20 + 3 : 0;
        }
        else
        {
            l_score = m_stock.peutConstruireMur() ? 5 + 3 : 0;
        }
        break;
    case Etape::EMPLACEMENT_TOUR:
        l_score = m_stock.peutConstruireTour() ? 5 + 1 : 0;
        break;
    case Etape::GRAAL:
        l_score = 5 + 5 * 4; // TODO: points of the Graal depend on its level (4 = on the robot)
        break;
    case Etape::POINT_PASSAGE:
        l_score = 0;
        break;
    default:
        return 0;
    }
    if (l_score > 0)
    {
        l_score += l_etape->getBoostManuelDeScore();
    }
    return l_score;
}

StockPierres& Coupe2027::getStock()
{
    return m_stock;
}

void Coupe2027::ramassePierres(Etape* e)
{
    if (e->getEtapeType() != Etape::CARRIERE_PIERRES)
    {
        std::cerr << "Not supposed to grab a pierre from here!" << std::endl;
        return;
    }

    CarrierePierres* l_carriere = static_cast<CarrierePierres*>(e->getAction());
    std::vector<Pierre> l_pierres = l_carriere->getPierres();

    // TODO: choose which actuator takes the pierres
    for (auto& l_actionneur : m_stock.getActionneurs())
    {
        if (l_actionneur.estVide() && !l_pierres.empty())
        {
            l_actionneur.ajoutePierres(l_pierres);
            l_carriere->retirePierres();
            l_pierres.clear();
        }
    }
    std::cout << "Pierres grabbed, new stock: " << m_stock.getNombrePierres() << std::endl;
}

int Coupe2027::construit(Etape* e)
{
    int l_scored = 0;
    EmplacementMur* l_mur;

    switch (e->getEtapeType())
    {
    case Etape::EMPLACEMENT_MUR:
        l_mur = static_cast<EmplacementMur*>(e->getAction());
        for (auto& l_actionneur : m_stock.getActionneurs())
        {
            if (l_mur->isPorte() && l_actionneur.peutConstruirePorte())
            {
                l_actionneur.vide();
                m_stock.setPorteConstruite(true);
                l_scored = 3 + 20;
                break;
            }
            if (!l_mur->isPorte() && l_actionneur.peutConstruireMur())
            {
                l_actionneur.vide();
                l_scored = 3 + 5;
                break;
            }
        }
        break;

    case Etape::EMPLACEMENT_TOUR:
        for (auto& l_actionneur : m_stock.getActionneurs())
        {
            if (l_actionneur.peutConstruireTour())
            {
                // TODO: take only one pierre from the actuator
                l_actionneur.vide();
                l_scored = 1 + 5;
                break;
            }
        }
        break;

    default:
        std::cerr << "Not supposed to build there! " << e->getEtapeType() << std::endl;
        return 0;
    }

    if (l_scored > 0)
    {
        // The spot is used: it becomes a regular waypoint
        e->finir();
    }
    return l_scored;
}
