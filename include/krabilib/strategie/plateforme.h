#pragma once

#include "krabilib/position.h"
#include "krabilib/pose.h"

class Plateforme
{
    public:
        Plateforme(){};
        Plateforme(Pose start_pose){};

    private:
        int m_current_level;
        Pose m_pose;
        Pose m_start_pose;
};