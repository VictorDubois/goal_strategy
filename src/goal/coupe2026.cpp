#include "goal_strategy/coupe2026.h"
#include "krabilib/pose.h"

#include <cmath>
#include <iostream>
#include <tf2_ros/transform_listener.h>

// todo delete this when strategie is done and it's not used anymore
Position Coupe2026::positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left)
{
    return positionC(1.5 - x_yellow_from_top_left, 1 - y_yellow_from_top_left);
}
//  ____________________________________________________
// |                       Grenier                    NID|
// |                                                     |
// |                                                     |
// |                        x                            |
// |                      <----o                         |
// |                           |                         |
// |                           |                         |
// |                           v y                       |
// |___________________________Thermometre______________ |
//                         PUBLIC
//
// "us" position are define as blue ("them" are yellow)

/**
 * @brief Initialize the graph of goals based on the color
 *
 * @param isYellow
 * @param starting_position
 */
Coupe2026::Coupe2026(const bool isYellow, rclcpp::Node::SharedPtr a_node)
  : StrategieV3(isYellow, true)
  , m_node(a_node)
{

    setRemainingTime(84 * 1000); // duration of the match

    m_stock.clear();

    // Initialisation des tableaux d'étapes
    m_tableau_etapes_total
      = Etape::initTableauEtapeTotal(NOMBRE_ETAPES); // new Etape*[NOMBRE_ETAPES];

    // Création des étapes
    // Les étapes qui correspondant à des actions sont créées automatiquement lors de l'ajout
    // d'actions

    int nid = Etape::makeEtape(positionC(-1.1f, -0.75f), "NID", Etape::DEPART);

    float reach = 0.22f; // Offset billig/centre de roues, en Y, en mètres
    float reachDepose = reach + 0.075f;
    float reachDeposeCentre = reach;
    float l_offset_GD = M_PI;          // actionneur à gauche = 0, à droite = M_PI
    float l_offset_billig_X = 0.0077f; // Offset billig/axes de roues, en X, en mètres
    float l_offset_BY = M_PI;          // blue = M_PI, yellow = 0
    if (isYellow)
    {
        l_offset_BY = 0;
        l_offset_billig_X *= -1;
    }

    // Definition des zone de ramassage
    int zone_de_ramassage_nid_petit_cote = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(-1.325f + reach, -0.2f + l_offset_billig_X), Angle(0 + l_offset_BY)),
        Pose(positionC(-1.325f, -0.2f), Angle(0 + l_offset_BY))),
      "zone_de_ramassage_nid_petit_cote");

    int point_passage_zone_de_ramassage_nid_petit_cote = Etape::makeEtape(
      positionC(-1.325f + reach, -0.4f + l_offset_billig_X), Etape::POINT_PASSAGE);

    int zone_de_ramassage_public_petit_cote = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(-1.325f + reach, 0.6f + l_offset_billig_X), Angle(0 + l_offset_BY)),
        Pose(positionC(-1.325f, 0.6f), Angle(0 + l_offset_BY))),
      "zone_de_ramassage_public_petit_cote");

    int zone_de_ramassage_public_grand_cote = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(-0.4f + l_offset_billig_X, 0.825f - reach), Angle(-M_PI / 2 + l_offset_GD)),
        Pose(positionC(-0.4f, 0.825f), Angle(-M_PI / 2 + l_offset_GD))),
      "zone_de_ramassage_public_petit_cote");

    int zone_de_ramassage_centre = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(-0.35f - l_offset_billig_X, 0.2f + reach), Angle(M_PI / 2 + l_offset_GD)),
        Pose(positionC(-0.35f, 0.2f), Angle(M_PI / 2 + l_offset_GD))),
      "zone_de_ramassage_public_petit_cote");

    int zone_de_ramassage_centre_from_top = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(-0.35f + l_offset_billig_X, 0.2f - reach), Angle(-M_PI / 2 + l_offset_GD)),
        Pose(positionC(-0.35f, 0.2f), Angle(-M_PI / 2 + l_offset_GD))),
      "zone_de_ramassage_centre_from_top");
    Etape::get(zone_de_ramassage_centre_from_top)
      ->addEtapeLieeParFinirEtape(zone_de_ramassage_centre);
    Etape::get(zone_de_ramassage_centre)
      ->addEtapeLieeParFinirEtape(zone_de_ramassage_centre_from_top);

    int point_passage_zone_de_ramassage_centre
      = Etape::makeEtape(positionC(-0.35f, 0.2f), Etape::POINT_PASSAGE_DESACTIVE);

    Etape::get(zone_de_ramassage_centre_from_top)
      ->addEtapeActiveApres(point_passage_zone_de_ramassage_centre);
    Etape::get(zone_de_ramassage_centre)
      ->addEtapeActiveApres(point_passage_zone_de_ramassage_centre);

    // ######### Autre cote ##############

    int zone_de_ramassage_nid_petit_cote_autre = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(1.325f - reach, -0.2f - l_offset_billig_X), Angle(M_PI + l_offset_BY)),
        Pose(positionC(1.325f, -0.2f), Angle(M_PI + l_offset_BY))),
      "zone_de_ramassage_nid_petit_cote");

    int zone_de_ramassage_public_petit_cote_autre = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(1.325f - reach, 0.6f - l_offset_billig_X), Angle(M_PI + l_offset_BY)),
        Pose(positionC(1.325f, 0.6f), Angle(M_PI + l_offset_BY))),
      "zone_de_ramassage_public_petit_cote");

    int zone_de_ramassage_public_grand_cote_autre = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(0.4f + l_offset_billig_X, 0.825f - reach), Angle(-M_PI / 2 + l_offset_GD)),
        Pose(positionC(0.4f, 0.825f), Angle(-M_PI / 2 + l_offset_GD))),
      "zone_de_ramassage_public_petit_cote");

    int zone_de_ramassage_centre_autre = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(0.35f - l_offset_billig_X, 0.2f + reach), Angle(M_PI / 2 + l_offset_GD)),
        Pose(positionC(0.35f, 0.2f), Angle(M_PI / 2 + l_offset_GD))),
      "zone_de_ramassage_public_petit_cote");

    int zone_de_ramassage_centre_autre_from_top = Etape::makeEtape(
      new ZoneDeRamassage(
        Pose(positionC(0.35f + l_offset_billig_X, 0.2f - reach), Angle(-M_PI / 2 + l_offset_GD)),
        Pose(positionC(0.35f, 0.2f), Angle(-M_PI / 2 + l_offset_GD))),
      "zone_de_ramassage_public_petit_cote_from_top");

    Etape::get(zone_de_ramassage_centre_autre_from_top)
      ->addEtapeLieeParFinirEtape(zone_de_ramassage_centre_autre);
    Etape::get(zone_de_ramassage_centre_autre)
      ->addEtapeLieeParFinirEtape(zone_de_ramassage_centre_autre_from_top);

    int point_passage_zone_de_ramassage_centre_autre
      = Etape::makeEtape(positionC(0.35f, 0.2f), Etape::POINT_PASSAGE_DESACTIVE);

    Etape::get(zone_de_ramassage_centre_autre_from_top)
      ->addEtapeActiveApres(point_passage_zone_de_ramassage_centre_autre);
    Etape::get(zone_de_ramassage_centre_autre)
      ->addEtapeActiveApres(point_passage_zone_de_ramassage_centre_autre);

    // Définition des garde mangers

    int garde_manger_petit_cote = Etape::makeEtape(
      new GardeManger(
        Pose(positionC(-1.4f + reachDepose, 0.2f + l_offset_billig_X), Angle(0 + l_offset_BY)),
        Pose(positionC(-1.4f, 0.2f), Angle(0 + l_offset_BY))),
      "garde_manger_petit_cote");

    int garde_manger_public_us = Etape::makeEtape(
      new GardeManger(Pose(positionC(-0.8f + l_offset_billig_X, 0.9f - reachDepose),
                           Angle(-M_PI / 2 + l_offset_GD)),
                      Pose(positionC(-0.8f, 0.9f), Angle(-M_PI / 2 + l_offset_GD))),
      "garde_manger_public_us");

    int garde_manger_public_milieu = Etape::makeEtape(
      new GardeManger(Pose(positionC(0.0f + l_offset_billig_X, 0.9f - reachDepose),
                           Angle(-M_PI / 2 + l_offset_GD)),
                      Pose(positionC(0.0f, 0.9f), Angle(-M_PI / 2 + l_offset_GD))),
      "garde_manger_public_milieu");

    int garde_manger_centre_us = Etape::makeEtape(
      new GardeManger(Pose(positionC(-0.7f - l_offset_billig_X, 0.2f + reachDeposeCentre),
                           Angle(M_PI / 2 + l_offset_GD)),
                      Pose(positionC(-0.7f, 0.2f), Angle(M_PI / 2 + l_offset_GD))),
      "garde_manger_centre_us");

    int garde_manger_centre_us_from_top = Etape::makeEtape(
      new GardeManger(Pose(positionC(-0.7f + l_offset_billig_X, 0.2f - reachDeposeCentre),
                           Angle(-M_PI / 2 + l_offset_GD)),
                      Pose(positionC(-0.7f, 0.2f), Angle(-M_PI / 2 + l_offset_GD))),
      "garde_manger_centre_us_from_top");

    Etape::get(garde_manger_centre_us_from_top)->addEtapeLieeParFinirEtape(garde_manger_centre_us);
    Etape::get(garde_manger_centre_us)->addEtapeLieeParFinirEtape(garde_manger_centre_us_from_top);

    int garde_manger_centre_milieu = Etape::makeEtape(
      new GardeManger(Pose(positionC(0.0f - l_offset_billig_X, 0.2f + reachDeposeCentre),
                           Angle(M_PI / 2 + l_offset_GD)),
                      Pose(positionC(0.0f, 0.2f), Angle(M_PI / 2 + l_offset_GD))),
      "garde_manger_centre_milieu");

    int garde_manger_centre_milieu_from_top = Etape::makeEtape(
      new GardeManger(Pose(positionC(0.0f + l_offset_billig_X, 0.2f - reachDeposeCentre),
                           Angle(-M_PI / 2 + l_offset_GD)),
                      Pose(positionC(0.0f, 0.2f), Angle(-M_PI / 2 + l_offset_GD))),
      "garde_manger_centre_milieu");

    Etape::get(garde_manger_centre_milieu_from_top)
      ->addEtapeLieeParFinirEtape(garde_manger_centre_milieu);
    Etape::get(garde_manger_centre_milieu)
      ->addEtapeLieeParFinirEtape(garde_manger_centre_milieu_from_top);

    [[maybe_unused]] int garde_manger_frigo_us = Etape::makeEtape(
      new GardeManger(Pose(positionC(-0.25f - l_offset_billig_X, -0.45f + reachDepose),
                           Angle((M_PI / 2) + l_offset_GD)),
                      Pose(positionC(-0.25f, -0.45f), Angle((M_PI / 2) + l_offset_GD))),
      "garde_manger_frigo_us");

    // ######### Autre cote ##############
    int garde_manger_petit_cote_them = Etape::makeEtape(
      new GardeManger(
        Pose(positionC(1.4f - reachDepose, 0.2f - l_offset_billig_X), Angle(M_PI + l_offset_BY)),
        Pose(positionC(1.4f, 0.2f), Angle(M_PI + l_offset_BY))),
      "garde_manger_petit_cote_them");

    int garde_manger_public_them = Etape::makeEtape(
      new GardeManger(Pose(positionC(0.8f + l_offset_billig_X, 0.9f - reachDepose),
                           Angle(-M_PI / 2 + l_offset_GD)),
                      Pose(positionC(0.8f, 0.9f), Angle(-M_PI / 2 + l_offset_GD))),
      "garde_manger_public_them");

    int garde_manger_centre_them = Etape::makeEtape(
      new GardeManger(Pose(positionC(0.7f - l_offset_billig_X, 0.2f + reachDeposeCentre),
                           Angle(M_PI / 2 + l_offset_GD)),
                      Pose(positionC(0.7f, 0.2f), Angle(M_PI / 2 + l_offset_GD))),
      "garde_manger_centre_them");

    int garde_manger_centre_them_from_top = Etape::makeEtape(
      new GardeManger(Pose(positionC(0.7f + l_offset_billig_X, 0.2f - reachDeposeCentre),
                           Angle(-M_PI / 2 + l_offset_GD)),
                      Pose(positionC(0.7f, 0.2f), Angle(-M_PI / 2 + l_offset_GD))),
      "garde_manger_centre_them_from_top");

    Etape::get(garde_manger_centre_them)
      ->addEtapeLieeParFinirEtape(garde_manger_centre_them_from_top);
    Etape::get(garde_manger_centre_them_from_top)
      ->addEtapeLieeParFinirEtape(garde_manger_centre_them);

    [[maybe_unused]] int garde_manger_frigo_them = Etape::makeEtape(
      new GardeManger(Pose(positionC(0.25f + l_offset_billig_X, -0.45f + reachDepose),
                           Angle(M_PI / 2 + l_offset_GD)),
                      Pose(positionC(0.25f, -0.45f), Angle(M_PI / 2 + l_offset_GD))),
      "garde_manger_frigo_them");

    // Thermometre
    int debut_thermometre = Etape::makeEtape(
      new Thermometre(Pose(positionC(-0.2f, 1.0f - 0.3f), Angle(M_PI / 2 + l_offset_GD)),
                      Pose(positionC(-0.2f, 1.0f), Angle(M_PI / 2 + l_offset_GD))),
      "debut_thermometre");
    int fin_thermometre = Etape::makeEtape(
      positionC(-0.8f, 1.0f - 0.3f), "fin_thermometre", Etape::EtapeType::POINT_PASSAGE);

    Etape::get(debut_thermometre)->setNumeroEtapeFinAction(fin_thermometre);

    // Définitions des voisins

    Etape::get(point_passage_zone_de_ramassage_nid_petit_cote)
      ->addVoisins(zone_de_ramassage_nid_petit_cote, nid);

    Etape::get(zone_de_ramassage_nid_petit_cote)->addVoisins(zone_de_ramassage_public_petit_cote);
    Etape::get(zone_de_ramassage_public_petit_cote)
      ->addVoisins(zone_de_ramassage_public_grand_cote);

    Etape::get(garde_manger_petit_cote)
      ->addVoisins(zone_de_ramassage_nid_petit_cote, zone_de_ramassage_public_petit_cote);
    Etape::get(garde_manger_public_us)
      ->addVoisins(garde_manger_petit_cote,
                   garde_manger_centre_us,
                   zone_de_ramassage_public_petit_cote,
                   zone_de_ramassage_public_grand_cote);
    Etape::get(garde_manger_centre_us)
      ->addVoisins(zone_de_ramassage_public_petit_cote,
                   zone_de_ramassage_public_grand_cote,
                   zone_de_ramassage_centre);

    Etape::get(garde_manger_public_milieu)
      ->addVoisins(zone_de_ramassage_centre,
                   zone_de_ramassage_centre_autre,
                   zone_de_ramassage_public_grand_cote,
                   zone_de_ramassage_public_grand_cote_autre);

    Etape::get(garde_manger_centre_milieu)
      ->addVoisins(zone_de_ramassage_centre,
                   zone_de_ramassage_centre_autre,
                   zone_de_ramassage_public_grand_cote,
                   zone_de_ramassage_public_grand_cote_autre);

    Etape::get(point_passage_zone_de_ramassage_centre)
      ->addVoisins(zone_de_ramassage_centre, zone_de_ramassage_centre_from_top);
    Etape::get(point_passage_zone_de_ramassage_centre_autre)
      ->addVoisins(zone_de_ramassage_centre_autre, zone_de_ramassage_centre_autre_from_top);

    // ######### Autre cote ##############
    Etape::get(garde_manger_petit_cote_them)
      ->addVoisins(zone_de_ramassage_nid_petit_cote_autre,
                   zone_de_ramassage_public_petit_cote_autre);

    Etape::get(zone_de_ramassage_public_grand_cote)
      ->addVoisins(zone_de_ramassage_centre, zone_de_ramassage_public_grand_cote_autre);
    Etape::get(zone_de_ramassage_public_grand_cote_autre)
      ->addVoisins(zone_de_ramassage_centre_autre, zone_de_ramassage_centre);
    Etape::get(zone_de_ramassage_public_petit_cote_autre)
      ->addVoisins(zone_de_ramassage_public_grand_cote_autre);
    Etape::get(zone_de_ramassage_nid_petit_cote_autre)
      ->addVoisins(zone_de_ramassage_public_petit_cote_autre);

    Etape::get(garde_manger_public_them)
      ->addVoisins(garde_manger_petit_cote_them,
                   garde_manger_centre_them,
                   zone_de_ramassage_public_petit_cote_autre,
                   zone_de_ramassage_public_grand_cote_autre);
    Etape::get(garde_manger_centre_them)
      ->addVoisins(zone_de_ramassage_public_petit_cote_autre,
                   zone_de_ramassage_public_grand_cote_autre,
                   zone_de_ramassage_centre_autre);

    // From top

    Etape::get(garde_manger_centre_us_from_top)
      ->addVoisins(zone_de_ramassage_nid_petit_cote, zone_de_ramassage_centre_from_top);

    Etape::get(garde_manger_centre_them_from_top)
      ->addVoisins(zone_de_ramassage_nid_petit_cote_autre, zone_de_ramassage_centre_autre_from_top);

    Etape::get(garde_manger_centre_milieu_from_top)
      ->addVoisins(zone_de_ramassage_centre_from_top, zone_de_ramassage_centre_autre_from_top);

    /*Etape::get(garde_manger_frigo_us)
      ->addVoisins(zone_de_ramassage_centre_from_top, zone_de_ramassage_centre_autre_from_top);
    Etape::get(garde_manger_frigo_them)
      ->addVoisins(zone_de_ramassage_centre_from_top, zone_de_ramassage_centre_autre_from_top);*/

    // Thermometre

    Etape::get(debut_thermometre)
      ->addVoisins(garde_manger_public_milieu,
                   zone_de_ramassage_public_grand_cote,
                   garde_manger_centre_milieu);

    Etape::get(fin_thermometre)
      ->addVoisins(garde_manger_centre_us, zone_de_ramassage_public_petit_cote, debut_thermometre);

    m_numero_etape_garage = nid; // Must be set!

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
void Coupe2026::etape_type_to_marker(visualization_msgs::msg::Marker& m, Etape* a_etape)
{
    std_msgs::msg::ColorRGBA yellow_color;
    yellow_color.r = 1.0f;
    yellow_color.g = 1.0f;
    yellow_color.b = 0.0f;
    yellow_color.a = 1.0f;

    std_msgs::msg::ColorRGBA blue_color;
    blue_color.r = 0.0f;
    blue_color.g = 0.0f;
    blue_color.b = 1.0f;
    blue_color.a = 1.0f;

    Position temp_pos;

    // auto& color = m.color;
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.type = visualization_msgs::msg::Marker::CUBE;

    std::vector<Caisse> m_stock;
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

    case Etape::EtapeType::ZONE_DE_RAMASSAGE:
        m.type = visualization_msgs::msg::Marker::CUBE_LIST;
        m.scale.z = 0.03f;
        m.color.a = 0.5f;
        m.scale.x = 0.15f;
        m.scale.y = 0.05f;

        temp_pos = Position(Distance(0.0), Distance(0.05f));

        m.points = { temp_pos * (1.5f), temp_pos * (0.5f), temp_pos * (-0.5), temp_pos * (-1.5f) };
        m_stock = static_cast<ZoneDeRamassage*>(a_etape->getAction())->getCaisses();

        m.colors = {
            yellow_color, blue_color, yellow_color, blue_color
        }; // Default config if no known color. Be be changed to grey once we have a measurement
           // system

        for (unsigned int i = 0; i < m_stock.size(); i++)
        {
            if (m_stock[i].getOrientation() == OrientationCaisse::our_side)
            {
                m.colors[i] = blue_color;
                if (isYellow())
                {
                    m.colors[i] = yellow_color;
                }
            }
            else if (m_stock[i].getOrientation() == OrientationCaisse::other_side)
            {
                m.colors[i] = yellow_color;
                if (isYellow())
                {
                    m.colors[i] = blue_color;
                }
            }
        }

        // Area blue
        m.color.r = 0.f / 255.f;
        m.color.g = 91.f / 255.f;
        m.color.b = 140.f / 255.f;

        break;
    case Etape::EtapeType::GARDE_MANGER:
        m.type = visualization_msgs::msg::Marker::CUBE; // ARROW to debug orientation
        m.scale.x = 0.2f;
        m.scale.y = 0.2f;
        m.scale.z = 0.01f;
        m.color.r = 222.0f / 255.f;
        m.color.g = 222.f / 255.f;
        m.color.b = 222.0f / 255.f;
        break;

    case Etape::EtapeType::THERMOMETRE:
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.scale.x = 0.1f;
        m.scale.y = 0.1f;
        m.scale.z = 0.01f;
        m.color.r = 255.0f / 255.f;
        m.color.g = 0.f / 255.f;
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
    default:
        break;
    }
}

/**
 * @brief Convert the graph into marker array for debug purpose
 *
 * @param ma marker array
 */
void Coupe2026::debugEtapes(visualization_msgs::msg::MarkerArray& ma)
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

            if (etape->getEtapeType() == Etape::EtapeType::ZONE_DE_RAMASSAGE)
            {
                ZoneDeRamassage* l_zdr = static_cast<ZoneDeRamassage*>(etape->getAction());
                m.pose = l_zdr->getAreaCenter();
            }
            else if (etape->getEtapeType() == Etape::EtapeType::GARDE_MANGER)
            {
                GardeManger* l_gardeManger = static_cast<GardeManger*>(etape->getAction());
                m.pose = l_gardeManger->getAreaCenter();
            }
            else if (etape->getEtapeType() == Etape::EtapeType::THERMOMETRE)
            {
                Thermometre* l_thermo = static_cast<Thermometre*>(etape->getAction());
                m.pose = l_thermo->getAreaCenter();
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
int Coupe2026::getScoreEtape(int i)
{
    RCLCPP_INFO_STREAM(m_node->get_logger(),
                       "getScoreEtape. Time: " << getRemainingTime() << std::endl);
    int l_score = 0;
    GardeManger* l_area;
    unsigned int l_stock_area;

    switch (this->m_tableau_etapes_total[i]->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::DEPART:
        l_score = 0;
        break;
    case Etape::ZONE_DE_RAMASSAGE:
        l_score = 3;

        if (m_stock.size() > 0)
        {
            l_score = -3;
        }
        break;
    case Etape::GARDE_MANGER:
        l_area = static_cast<GardeManger*>(this->m_tableau_etapes_total[i]->getAction());
        l_stock_area = l_area->getCaisses().size();

        if (m_stock.size() && l_stock_area == 0)
        {
            l_score = 6;
            if (l_stock_area <= 4)
            {
                l_score = 12;
            }
            if (l_stock_area == 0)
            {
                l_score = 24;
            }
        }
        break;
    case Etape::THERMOMETRE:
        // l_score = 10;
        break;
    case Etape::POINT_PASSAGE:
        l_score = 0;
        break;
    default:
        return 0;
    }
    return l_score;
}

std::vector<Caisse> Coupe2026::getStock()
{
    return m_stock;
}

void Coupe2026::grabCaisses(Etape* e)
{
    ZoneDeRamassage* l_zone_ramassage;
    std::vector<Caisse> l_caisses;

    switch (e->getEtapeType())
    {
    // A faire : remplacer la priorite par le nombre de points obtenables a l'etape
    case Etape::ZONE_DE_RAMASSAGE:
        l_zone_ramassage = static_cast<ZoneDeRamassage*>(e->getAction());
        l_caisses = l_zone_ramassage->getCaisses();

        m_stock.insert(m_stock.end(), l_caisses.begin(), l_caisses.end());
        std::cout << "Caisses grabbed, new stock: " << m_stock.size() << std::endl;

        break;

    default:
        std::cerr << "Not supposed to grab a caisse from here!" << std::endl;
    }
}

int Coupe2026::dropCaisses(Etape* e)
{
    int l_scored = 0;
    GardeManger* l_area;
    std::vector<Caisse> l_stock_area;
    switch (e->getEtapeType())
    {
        // A faire : remplacer la priorite par le nombre de points obtenables a l'etape

    case Etape::GARDE_MANGER:
        l_area = static_cast<GardeManger*>(e->getAction());

        if (m_stock.size())
        {
            l_area->addCaisse(m_stock.back());
            // m_stock.pop_back();
            m_stock.clear();

            l_scored = 9;
        }
        break;

    default:
        std::cerr << "Not supposed to drop a caisse there! " << e->getEtapeType() << std::endl;
    }

    return l_scored;
}