#include "Krabi/strategie/accelerator.h"
#include "Krabi/strategie/mediumLevelAction.h"
#include "Krabi/position.h"

#ifndef STANDALONE_STRATEGIE
#include "Krabi/strategieV2.h"
#include "Krabi/command.h"
#endif

#ifdef QTGUI
#include <QDebug>
#endif

Accelerator::Accelerator(){}

Accelerator::Accelerator(Position goalPosition):MediumLevelAction(goalPosition)
{
    goalPosition = this->goalPosition;
    position_depart = goalPosition;
}

Accelerator::~Accelerator(){}

Etape::EtapeType Accelerator::getType()
{
    return Etape::ACCELERATOR;
}

int Accelerator::update()
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
        qDebug() << "On se prepare a pousser l'accelerator";
#endif
     status++;
    }

    else if (status == 1) {
        if (Command::isNear(this->getGoalPosition(), 200.0f)) // le second paramètre est la distance a l'objectif
        {
            // après avoir poussé les cubes on revient en marche arrière
            StrategieV2::setCurrentGoal(position_depart, true, VITESSE_LINEAIRE_MAX, -100.0, 10.f);
#ifdef QTGUI
        qDebug() << "On revient en marche arriere au point ou on prend son elan";
#endif
            status++;
        }
    }

    else if (status == 2) {

        if (Command::isNear(position_depart, 10.0f)) // le second paramètre est la distance a l'objectif
        {
#ifdef QTGUI
        qDebug() << "On est revenu au point ou on prend son elan";
#endif
            status++;
        }
    }

    else if (status == 3) {
#ifdef QTGUI
        qDebug() << "Etape fermeture de l'accelerator finie";
#endif
        status = -1;
    }


#endif

    return status;
}
