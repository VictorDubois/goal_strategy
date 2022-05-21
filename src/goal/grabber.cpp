#include <goal_strategy/grabber.h>

Grabber::Grabber(Position a_relative_position,
                 std::shared_ptr<Servomotor> a_servo_base,
                 std::shared_ptr<Servomotor> a_servo_mid,
                 std::shared_ptr<Servomotor> a_servo_suction_cup,
                 std::shared_ptr<Pump> a_pump)
  : m_relative_position(a_relative_position)
  , m_servo_base(a_servo_base)
  , m_servo_mid(a_servo_mid)
  , m_servo_suction_cup(a_servo_suction_cup)
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

void Grabber::release_hexagon_on_ground(GrabberContent type_to_release)
{
    if (type_to_release == m_content || type_to_release == GrabberContent::ANY)
    {
        release_hexagon_on_ground();
    }
}

void Grabber::release_hexagon_on_ground()
{
    // do release
    m_servo_base->setAngle(150);
    m_servo_base->setSpeed(128);
    m_servo_mid->setAngle(25);
    m_servo_mid->setSpeed(128);
    m_servo_suction_cup->setAngle(160);
    m_servo_suction_cup->setSpeed(128);
    sleep(2);

    m_pump->setPumping(false);
    m_pump->release();
    usleep(500000);

    m_servo_base->setAngle(50);
    m_servo_base->setSpeed(128);
    m_servo_mid->setAngle(30);
    m_servo_mid->setSpeed(128);
    m_servo_suction_cup->setAngle(160);
    m_servo_suction_cup->setSpeed(128);
    usleep(500000);
}

void Grabber::grab_hexagon(GrabberContent type_to_catch)
{
    grab_hexagon();
    m_content = type_to_catch;
}

void Grabber::grab_hexagon()
{
    // do catch
    m_servo_base->setAngle(150);
    m_servo_base->setSpeed(128);
    m_servo_mid->setAngle(25);
    m_servo_mid->setSpeed(128);
    m_servo_suction_cup->setAngle(160);
    m_servo_suction_cup->setSpeed(128);
    sleep(2);

    m_pump->setPumping(true);
    usleep(500000);

    m_servo_base->setAngle(50);
    m_servo_base->setSpeed(5);
    m_servo_mid->setAngle(30);
    m_servo_mid->setSpeed(5);
    m_servo_suction_cup->setAngle(160);
    m_servo_suction_cup->setSpeed(5);
    usleep(500000);
}

void Grabber::grab_statuette()
{
    // do catch
    m_servo_base->setAngle(55);
    m_servo_mid->setAngle(140);
    m_servo_suction_cup->setAngle(90);

    m_servo_base->setSpeed(128);
    m_servo_mid->setSpeed(128);
    m_servo_suction_cup->setSpeed(128);
    sleep(2);
    m_pump->setPumping(true);

    m_servo_base->setAngle(90);
    m_servo_mid->setAngle(100);
    m_servo_suction_cup->setAngle(90);

    m_servo_base->setSpeed(5);  
    m_servo_mid->setSpeed(5);    
    m_servo_suction_cup->setSpeed(5);
    sleep(2);

    m_servo_base->setAngle(50);
    m_servo_base->setSpeed(5);
    m_servo_mid->setAngle(30);
    m_servo_mid->setSpeed(5);
    m_servo_suction_cup->setAngle(160);
    m_servo_suction_cup->setSpeed(5);
    usleep(500000);
}

void Grabber::release_statuette()
{
    // do release
    m_servo_base->setAngle(65);
    m_servo_base->setSpeed(128);
    m_servo_mid->setAngle(100);
    m_servo_mid->setSpeed(128);
    m_servo_suction_cup->setAngle(90);
    m_servo_suction_cup->setSpeed(128);
    sleep(2);

    m_pump->setPumping(false);
    m_pump->release();
    usleep(500000);

    m_servo_base->setAngle(50);
    m_servo_base->setSpeed(128);
    m_servo_mid->setAngle(30);
    m_servo_mid->setSpeed(128);
    m_servo_suction_cup->setAngle(160);
    m_servo_suction_cup->setSpeed(128);
    usleep(500000);
}
