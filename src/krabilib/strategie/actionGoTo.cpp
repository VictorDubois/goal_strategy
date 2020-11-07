#include "krabilib/strategie/actionGoTo.h"

#ifdef QTGUI
	#include <QDebug>
    #define abs fabs
#endif

#ifndef STANDALONE_STRATEGIE
	#include "krabilib/odometrie.h"
	#include "krabilib/strategieV2.h"
	#include "krabilib/sharpSensor.h"
	#include "krabilib/leds.h"
#endif


/** valeurs :
    1 : aller ver le haut (0,Y)
    2 : aller vers le bas (0, -Y)
    3 : aller vers la droite (X, 0)
    4 : aller vers la gauche (-X, 0)
    5 : aller bas+droite(X,-Y)
    6 : aller bas+gauche(-X, -Y)
    7 : aller haut+droit(X, Y)
    8 : aller haut+gauche(-X,Y)
**/
/*
uint8_t tableauEvitement[20][30] =
    { //0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29
        3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, // 019
        2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 3, 3, 3, 4, 4, 4, 4, 4, 4, 2, // 18
        2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 3, 3, 3, 4, 4, 4, 4, 4, 2, 2, // 17
        2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 5, 5, 5, 5, 3, 3, 3, 3, 4, 4, 4, 4, 2, 2, 2, // 16
        2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 6, 6, 5, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, // 15
        2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 6, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, // 14
        2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, // 13
        2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, // 12
        2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, // 11
        2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, // 10
        1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, // 9
        1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, // 8
        1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, // 7
        1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, // 6
        1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, // 5
        1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, // 4
        1, 1, 1, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 1, 1, 1, // 3
        1, 1, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 1, 1, // 2
        1, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 1, // 1
        3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, // 0
    };//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29
    */
ActionGoTo::ActionGoTo(Position goalPos, bool a_goBack, float _precision, float stopAtDistance)
    : MediumLevelAction(goalPos)
{
    command = 0;
    smoothFactor = 200.f;
    stopAtDistance = stopAtDistance;
    goBack = a_goBack;
    goalAngle = 0;
    goingCurve = false;
    curveFactor = 1;
    precision = _precision;
}

ActionGoTo::~ActionGoTo()
{
    //dtor
}

int ActionGoTo::update()
{
#ifdef STANDALONE_STRATEGIE
    return 0;
#else

    if (status == 0)
    {
#ifdef QTGUI
        qDebug() << "actionGoTo " << goalPosition.getX() << " " << goalPosition.getY();
#endif
        //allumerLED2();
        Position pos = Odometrie::odometrie->getPos().getPosition();
        //Position vect = goalPosition - pos;
        //vect *= (1.f/vect.getNorme());

        goingCurve = false;//StrategieV2::getJustAvoided();

        /*if (goingCurve)
        {
            float currentAngle = wrapAngle(Odometrie::odometrie->getPos().getAngle());
            intermediateGoalPosition = Odometrie::odometrie->getPos().getPosition()+Position(-200*cos(currentAngle), 200*abs(currentAngle));
            StrategieV2::setCurrentGoal(intermediateGoalPosition, true); // a changer selon le servo qui détecte
            status = 1;
        }
        else*/
        {
            if (nextGoal == Position())
                command = StrategieV2::setCurrentGoal(goalPosition, goBack, VITESSE_LINEAIRE_MAX, -100.0, stopAtDistance);
            else
                command = StrategieV2::setCurrentGoalSmooth(goalPosition, nextGoal, smoothFactor, goBack);
            status = 3;
        }

    }
    else if (status ==1) // on recule
    {
        Vec2d vect = intermediateGoalPosition - Odometrie::odometrie->getPos().getPosition();
        //std::cout << "status = 1 " << vect.getNorme() << std::endl;
        if (vect.getNorme() < precision) // now we have
        {
            /*
            Position pos = Odometrie::odometrie->getPos().getPosition();
            Position vect = goalPosition - pos;
            float distance = vect.getNorme();
            vect /= distance;
            curveFactor = distance/50;
            goalAngle = wrapAngle(M_PI*10.f/(float)(curveFactor)+vect.getAngle());
            StrategieV2::lookAt(pos+Position(100*cos(goalAngle),100*sin(goalAngle)));
            status = 2;*/
            /*vect = goalPosition-intermediateGoalPosition;
            Position vect2 = vect;
            vect2 /= vect2.getNorme();
            int sign = 0;
            if (vect.getX() > 0 && intermediateGoalPosition.getY() < 800)
                sign = 1;
            else if (vect.getX() > 0 && intermediateGoalPosition.getY() >= 800)
                sign = -1;
            else if (vect.getX() < 0)
            vect = vect + Position(500*vect.getY(), 500*vect.getX());*/
        }
    }
    else if (status == 2)
    {
        //std::cout << "status = 2" << std::endl;
        /*float currentAngle = wrapAngle(Odometrie::odometrie->getPos().getAngle());
        if (fabs(currentAngle - goalAngle) < 0.02)
        {
            if (goingCurve)
            {
                Position currentPos = Odometrie::odometrie->getPos().getPosition();
                Position moitie = (goalPosition-currentPos);
                moitie *= 0.5;
                Position vect = moitie;
                vect *= (1.f/vect.getNorme());
                //std::cout << vect.getX() << " " << vect.getY() << std::endl;

                moitie += Position(curveFactor*20*vect.getY(),curveFactor*(-20)*vect.getX()); // on déplace le point pour qu'il soit décalé du milieu

                StrategieV2::setCurrentGoal(goalPosition, currentPos + moitie, 1, false);
                StrategieV2::setJustAvoided(false);
                status = 3;
            }
            else
            {
                StrategieV2::setCurrentGoal(goalPosition, goBack);
                status = 3;
            }
        }*/
    }
    else if (status == 3)
    {
//        allumerLED();
        //std::cout << "status = 3" << std::endl;
        Vec2d vect = goalPosition - Odometrie::odometrie->getPos().getPosition();
        //std::cout << vect.getNorme() << std::endl;
        //std::cout << Odometrie::odometrie->getPos().getPosition().getX() << " "<< Odometrie::odometrie->getPos().getPosition().getY()   << std::endl;
        if (vect.getNorme() < precision || (command != 0 && command->fini()))
            status = -1;
    }
    return status;
#endif
}

void ActionGoTo::collisionAvoided()
{
    //if (status > 0)
        this->status = 0;
}

void ActionGoTo::reset()
{
    this->status = 0;
}

void ActionGoTo::setNextGoal(Position nextGoal)
{
    this->nextGoal = nextGoal;
}
