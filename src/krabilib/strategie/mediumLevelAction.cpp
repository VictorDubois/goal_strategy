#include "krabilib/strategie/mediumLevelAction.h"

MediumLevelAction::MediumLevelAction(Position goalPos, bool goBack)
  : goalPosition(goalPos)
  , status(0)
  , goBack(goBack)
{
    //(StrategieV2::getIsBlue() ? goalPos : Position(3000,0)-goalPos);
}

MediumLevelAction::~MediumLevelAction()
{
}

Position MediumLevelAction::getGoalPosition()
{
    return goalPosition;
}

bool MediumLevelAction::getGoBack()
{
    return goBack;
}

void MediumLevelAction::collisionAvoided()
{
    status = 0; // reinitialize the status if function not overloaded
}

void MediumLevelAction::setGoBack(bool val)
{
    goBack = val;
}

void MediumLevelAction::reset()
{
}

Etape::EtapeType MediumLevelAction::getType()
{
    return Etape::POINT_PASSAGE;
}

void MediumLevelAction::updateTime(int millisecondesRestantes)
{
    this->millisecondesRestantes = millisecondesRestantes;
}
