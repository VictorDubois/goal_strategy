#pragma once

#include "krabilib/pose.h"
#include "krabilib/position.h"

class Plateforme
{
public:
    Plateforme() {};
    Plateforme(Pose /*start_pose*/) {};

private:
    int m_current_level;
    Pose m_pose;
    Pose m_start_pose;
};