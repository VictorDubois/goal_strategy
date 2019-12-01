#ifndef ACTIONGOTO_H_INCLUDED
#define ACTIONGOTO_H_INCLUDED

#include "Krabi/strategie/mediumLevelAction.h"
#include "Krabi/position.h"
//#include <stdint.h>

class Command;

class ActionGoTo : public MediumLevelAction
{
    public:
        ActionGoTo(Position goalPos = Position(), bool goBack1 = false, float _precision = 30.f, float stopAtDistance = 0.f);
        virtual ~ActionGoTo();

        virtual int update();
        virtual void collisionAvoided();
        void reset();

        void setNextGoal(Position nextGoal);
    protected:
    private:
        float goalAngle, precision, stopAtDistance;
        bool goingCurve;
        int curveFactor, smoothFactor;

        Position intermediateGoalPosition, nextGoal;
        Command* command;
};

#endif // ACTIONGOTO_H_INCLUDED
