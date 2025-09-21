#pragma once

#define NOMBRE_ETAPES 50

#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/gardeManger.h"
#include "krabilib/strategie/strategiev3.h"
#include "krabilib/strategie/thermometre.h"
#include "krabilib/strategie/zoneDeRamassage.h"
#include <geometry_msgs/msg/point.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class Coupe2026 : public StrategieV3
{
public:
    Coupe2026(bool isYellow);
    void debugEtapes(visualization_msgs::msg::MarkerArray& ma);
    void etape_type_to_marker(visualization_msgs::msg::Marker& m, Etape* e);

    void grabPlateformes(Etape*) {}; // 2025 only
    int dropPlateformes(Etape*)
    {
        return 0;
    }; // 2025 only

    void grabCaisses(Etape* e); // 2026 only
    int dropCaisses(Etape* e);  // 2026 only

    void grabThermometre(Etape* e); // 2026 only
    void dropThermometre(Etape* e); // 2026 only

    Position positionCAbsolute(double x_yellow_from_top_left, double y_yellow_from_top_left);

    std::vector<Caisse> getStock();

    Position getParkedPosition()
    {
        return positionC(-1.35f, -0.85f);
    };

private:
    int getScoreEtape(int i);

    uint32_t m_seq;

    std::vector<Caisse> m_stock;
};
