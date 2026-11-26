#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"

/** @brief Emplacement de mur (5 per team, 200 x 400 mm): needs a full stack of 3 pierres. Can also host the porte (max 1 porte per team, tracked in StockPierres). */
class EmplacementMur : public MediumLevelAction
{
public:
    EmplacementMur();

    EmplacementMur(Pose goalPose, Pose area_center);
    EmplacementMur(Pose goalPose);

    ~EmplacementMur();

    int update();

    Etape::EtapeType getType();

    Pose getAreaCenter();
    Position getGoalPosition();
    Pose getGoalPose();

    /** @brief True if the porte is to be built here instead of a mur (3 pierres: 2 verticales + 1 horizontale) */
    bool isPorte() const;
    void setPorte(bool a_porte);

protected:
    Pose m_goal_pose;
    Pose m_area_center;
    bool m_porte = false;
};
