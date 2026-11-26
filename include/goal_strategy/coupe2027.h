#pragma once

#define NOMBRE_ETAPES 50

#include "krabilib/strategie/carrierePierres.h"
#include "krabilib/strategie/constructionMur.h"
#include "krabilib/strategie/constructionTour.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/graal.h"
#include "krabilib/strategie/stockPierres.h"
#include "krabilib/strategie/strategiev3.h"
#include <geometry_msgs/msg/point.hpp>

#include "rclcpp/node.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/** @brief The robot must be back in its base (where the opponent PAMIs cannot enter) before the
 * PAMIs attack phase starts (85 s after the start of the match). TODO: choose a margin. */
#define TEMPS_RETOUR_BASE_MS (85 * 1000)

/** @brief Total duration of the match. TODO: check the rules (the attack phase starts at 85 s). */
#define DUREE_MATCH_MS (100 * 1000)

class Coupe2027 : public StrategieV3
{
public:
    Coupe2027(bool isYellow, rclcpp::Node::SharedPtr a_node);
    void debugEtapes(visualization_msgs::msg::MarkerArray& ma);
    void etape_type_to_marker(visualization_msgs::msg::Marker& m, Etape* e);

    /** @brief Add the pierres of the carriere to the stock of the robot */
    void ramassePierres(Etape* e);
    /** @brief Build a mur, a tour or a porte with the stock. Returns the points scored */
    int construit(Etape* e);

    Position positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left);

    StockPierres& getStock();

    /** @brief Position of the base (salle du trone) */
    Position getParkedPosition()
    {
        // TODO: the robot will need to go around the built castle
        return positionC(-1.25f, -0.0f);
    };

    /** @brief True if the PAMIs of the opponent can attack (the robot must be in its base) */
    bool isTempsRetourBase();

private:
    int getScoreEtape(int i);

    uint32_t m_seq;

    StockPierres m_stock;

    rclcpp::Node::SharedPtr m_node;
};
