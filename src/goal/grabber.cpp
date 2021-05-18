#include <goal_strategy/grabber.h>

Grabber::Grabber(Position a_relative_position,
                 std::shared_ptr<Servomotor> a_servo,
                 std::shared_ptr<Pump> a_pump)
  : m_relative_position(a_relative_position)
  , m_servo(a_servo)
  , m_pump(a_pump)
{
    m_content = GrabberContent::NOTHING;
}

/*
 * Returns the reach of the grabber, with respect to the center of the robot
 */
float Grabber::getReach()
{
    return m_relative_position.getNorme();
}

Angle Grabber::getAngle()
{
    return m_relative_position.getAngle();
}

void Grabber::release(GrabberContent type_to_release)
{
    if (type_to_release == m_content || type_to_release == GrabberContent::ANY)
    {
        release();
    }
}

void Grabber::release()
{
    // do release
    m_servo->setAngle(50);
    m_servo->setSpeed(10);

    sleep(2);

    m_pump->setPumping(false);
    m_servo->setAngle(120);
    m_servo->setSpeed(10);
}

void Grabber::grab(GrabberContent type_to_catch)
{
    grab();
    m_content = type_to_catch;
}

void Grabber::grab()
{
    // do catch
    m_servo->setAngle(50);
    m_servo->setSpeed(10);
    m_pump->setPumping(true);

    sleep(2);

    m_servo->setAngle(120);
    m_servo->setSpeed(10);
}
