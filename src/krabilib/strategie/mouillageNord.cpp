#include "krabilib/strategie/mouillageNord.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/position.h"


#ifndef STANDALONE_STRATEGIE
#include "krabilib/strategie/command.h"
#include "krabilib/strategie/strategieV2.h"
//#define VITESSE_LINEAIRE_MAX 100
#endif



#ifdef QTGUI
#include <QDebug>
#endif

#define QTGUI

#define qDebug() std::cout

MouillageNord::MouillageNord(){}

MouillageNord::MouillageNord(Position goalPosition):MediumLevelAction(goalPosition)
{
    goalPosition = this->goalPosition;
    position_depart = goalPosition;
}

MouillageNord::~MouillageNord(){}

Etape::EtapeType MouillageNord::getType()
{
    return Etape::MOUILLAGE_NORD;
}

int MouillageNord::update()
{

    // la stratégie standalone ne s'occupe pas de faire tourner les actions
#ifdef STANDALONE_STRATEGIE
    // Finir tout de suite l'action
    status = -1;
#else

    if (status == 0) //Début
    {

        StrategieV2::setCurrentGoal(this->getGoalPosition(), false, VITESSE_LINEAIRE_MAX, -100.0, 200.f);
#ifdef QTGUI
        qDebug() << "On arrive devant le mouillageNord";
#endif
     status++;
    }

    else if (status == 1) {
        if (Command::isNear(this->getGoalPosition(), 10.0f)) // le second paramètre est la distance a l'objectif
        {
#ifdef QTGUI
        qDebug() << "Etape mouillageNord finie";
#endif
        status = -1;
       }
    }


#endif

    return status;
}
