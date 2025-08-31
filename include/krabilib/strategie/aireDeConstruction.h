#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/stockDeMatierePremiere.h" // pour def plateforme

#ifndef STANDALONE_STRATEGIE
#include "krabilib/command.h"
#endif // STANDALONE_STRATEGIE

enum AireSize
{
    AIRE_BIG,
    AIRE_SMALL
};

enum Owner
{
    us,
    them
};

class AireDeConstruction : public MediumLevelAction
{
public:
    AireDeConstruction();

    AireDeConstruction(Pose goalPose, Pose area_center, Owner us_or_them, AireSize a_size);
    AireDeConstruction(Pose goalPose, Owner us_or_them, AireSize a_size);

    ~AireDeConstruction();

    int update();

    Etape::EtapeType getType();

    Owner getOwner();

    void addPlateforme(Plateforme added_plateforme);

    std::vector<Plateforme> getPlateformes();

    Pose getAreaCenter();
    Position getGoalPosition();
    Pose getGoalPose();

    bool isBig()
    {
        return m_size == AireSize::AIRE_BIG;
    }

protected:
    Pose m_goal_pose;
    Pose m_area_center;
    Owner m_us_or_them;
    std::vector<Plateforme> m_stock;
    AireSize m_size;
};
