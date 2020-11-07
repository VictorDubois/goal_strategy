#include "krabilib/strategie/mancheAAir.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/position.h"


#ifndef STANDALONE_STRATEGIE
#include "krabilib/strategie/command.h"
#include "krabilib/strategie/strategieV2.h"
#endif



#ifdef QTGUI
#include <QDebug>
#endif

#define QTGUI

#define qDebug() std::cout

MancheAAir::MancheAAir(){}

MancheAAir::MancheAAir(Position goalPosition):MediumLevelAction(goalPosition)
{
    goalPosition = this->goalPosition;
    position_depart = goalPosition;
}

MancheAAir::~MancheAAir(){}

Etape::EtapeType MancheAAir::getType()
{
    return Etape::MANCHE_A_AIR;
}

int MancheAAir::update()
{

    // la stratégie standalone ne s'occupe pas de faire tourner les actions
#ifdef STANDALONE_STRATEGIE
    // Finir tout de suite l'action
    status = -1;
#else
    if (status == 0) //Début
    {
        // A faire : Desactiver le Sharp avant

        StrategieV2::setCurrentGoal(this->getGoalPosition(), false, VITESSE_LINEAIRE_MAX, -100.0, 200.f);
#ifdef QTGUI
        qDebug() << "On arrive devant le phare";
#endif
     status++;
    }

    else if (status == 1) {
        if (Command::isNear(this->getGoalPosition(), 10.0f)) // le second paramètre est la distance a l'objectif
        {
            // On s'oriente pour le bras
            StrategieV2::lookAt(45 * M_PI/180.f);

#ifdef QTGUI
            qDebug() << "On tourne pour mettre le bras face au phare";
#endif
            status++;
        }
    }

    else if (status == 2) {

        if (Command::isLookingAt(45 * M_PI/180.f, 5.f * M_PI/180.f)) // le second paramètre est l'angle a l'objectif
        {
#ifdef QTGUI
        qDebug() << "On baisse le bras";
#endif
            status++;
        }
    }
    else if (status < 10) {

#ifdef QTGUI
        qDebug() << "Manche à air relevé";
        qDebug() << "On remonte le bras";
#endif
            status++;
        }

    else if (status == 20) {
#ifdef QTGUI
        qDebug() << "Etape manche a air finie";
#endif
        status = -1;
    }


#endif

    return status;
}
