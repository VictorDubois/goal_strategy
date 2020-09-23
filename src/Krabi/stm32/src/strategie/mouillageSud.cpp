#include "Krabi/strategie/mouillageSud.h"
#include "Krabi/strategie/mediumLevelAction.h"
#include "Krabi/position.h"


#ifndef STANDALONE_STRATEGIE
#include "Krabi/strategie/command.h"
#include "Krabi/strategie/strategieV2.h"
//#define VITESSE_LINEAIRE_MAX 100
#endif



#ifdef QTGUI
#include <QDebug>
#endif

#define QTGUI

#define qDebug() std::cout

MouillageSud::MouillageSud(){}

MouillageSud::MouillageSud(Position goalPosition):MediumLevelAction(goalPosition)
{
    goalPosition = this->goalPosition;
    position_depart = goalPosition;
}

MouillageSud::~MouillageSud(){}

Etape::EtapeType MouillageSud::getType()
{
    return Etape::MOUILLAGE_SUD;
}

int MouillageSud::update()
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
        qDebug() << "On arrive devant le mouillageSud";
#endif
     status++;
    }

    else if (status == 1) {
        if (Command::isNear(this->getGoalPosition(), 10.0f)) // le second paramètre est la distance a l'objectif
        {
#ifdef QTGUI
        qDebug() << "Etape mouillageSud finie";
#endif
        status = -1;
       }
    }


#endif

    return status;
}
