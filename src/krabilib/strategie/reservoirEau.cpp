#include "krabilib/strategie/reservoirEau.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/position.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/strategieV2.h"
#include "krabilib/command.h"
#endif

#ifdef QTGUI
#include <QDebug>
#endif

ReservoirEau::ReservoirEau(){}

ReservoirEau::ReservoirEau(Position goalPosition):MediumLevelAction(goalPosition)
{
    goalPosition = this->goalPosition;
    position_depart = goalPosition;
}

ReservoirEau::~ReservoirEau(){}

Etape::EtapeType ReservoirEau::getType()
{
    return Etape::RESERVOIR_EAU;
}

int ReservoirEau::update()
{

    // la stratégie standalone ne s'occupe pas de faire tourner les actions
#ifdef STANDALONE_STRATEGIE
    // Finir tout de suite l'action
    status = -1;
#else

    if (status == 0) //Début
    {
        // A faire : Desactiver le Sharp avant

        StrategieV2::setCurrentGoal(this->getGoalPosition(), false, VITESSE_LINEAIRE_MAX, -100.0, 10.f);
#ifdef QTGUI
        qDebug() << "On se dirige vers le ReservoirEau";
#endif
        status++;
    }

    else if (status == 1) {

        if (Command::isNear(position_depart, 10.0f)) // le second paramètre est la distance a l'objectif
        {
#ifdef QTGUI
        qDebug() << "Reservoir a eau atteint";
#endif
            status++;
        }
    }

    else if (status == 2) {
#ifdef QTGUI
        qDebug() << "Etape reservoir eau finie";
#endif
        status = -1;
    }


#endif

    return status;
}
