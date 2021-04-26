#include <goal_strategy/grabber.h>

Grabber::Grabber(Position a_relative_position)
  : relative_position(a_relative_position)
{
    content = GrabberContent::NOTHING;
}

/*
 * Returns the reach of the grabber, with respect to the center of the robot
 */
float Grabber::getReach()
{
    return this->relative_position.getNorme();
}

Angle Grabber::getAngle()
{
    return this->relative_position.getAngle();
}

void Grabber::release(GrabberContent type_to_release)
{
    if (type_to_release == this->content || type_to_release == GrabberContent::ANY)
    {
        this->release();
    }
}

void Grabber::release()
{
    // do release
}

void Grabber::grab(GrabberContent type_to_catch)
{
    grab();
    this->content = type_to_catch;
}

void Grabber::grab()
{
    // do catch
}
