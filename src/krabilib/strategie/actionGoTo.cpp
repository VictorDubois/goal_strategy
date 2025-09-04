#include "krabilib/strategie/actionGoTo.h"

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
    // dtor
}

int ActionGoTo::update()
{
    return 0;
}

void ActionGoTo::collisionAvoided()
{
    // if (status > 0)
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
