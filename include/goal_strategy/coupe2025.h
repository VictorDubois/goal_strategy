#pragma once

#define NOMBRE_ETAPES 50

#include "krabilib/strategie/aireDeConstruction.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/stockDeMatierePremiere.h"
#include "krabilib/strategie/strategiev3.h"
#include <geometry_msgs/msg/point.hpp>

#include "rclcpp/node.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

enum StartingPosition2025
{
    FRONT_START,
    COTE_START,
    PAMI_START
};

class Coupe2025 : public StrategieV3
{
public:
    Coupe2025(bool isYellow,
              StartingPosition2025 starting_position,
              rclcpp::Node::SharedPtr a_node);
    void debugEtapes(visualization_msgs::msg::MarkerArray& ma);
    void etape_type_to_marker(visualization_msgs::msg::Marker& m, Etape* e);

    void grabPlateformes(Etape* e); // 2025 only
    int dropPlateformes(Etape* e);  // 2025 only

    void dropBanner(Etape* e); // 2025 only

    Position positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left);

    std::vector<Plateforme> getStock();

    Position getParkedPosition()
    {
        return positionC(-1.1f, -0.480f);
    };

private:
    int getScoreEtape(int i);

    uint32_t m_seq;

    std::vector<Plateforme> m_stock;

    rclcpp::Node::SharedPtr m_node;
};
