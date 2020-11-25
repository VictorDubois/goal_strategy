#include <math.h>

#include "krabilib/pose.h"
#ifdef USE_ROS
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"
#endif

Pose::Pose()
{
}

Pose::Pose(const Position& pos, Angle ang)
  : m_position(pos)
  , m_angle(ang)
{
}

Pose::Pose(const Pose& original)
  : m_position(original.m_position)
  , m_angle(original.m_angle)
{
}

Pose Pose::operator+(Distance distance) const
{
    return Pose(Position({ m_position.getX() + distance * cos(m_angle),
                           m_position.getY() + distance * sin(m_angle) }),
                m_angle);
}

Pose Pose::operator-(Distance distance) const
{
    return operator+(Distance(-distance));
}

const Pose& Pose::operator=(const Pose& p)
{
    m_position = p.m_position;
    m_angle = p.m_angle;

    return *this;
}

bool Pose::operator==(const Pose& p) const
{
    return ((m_position == p.m_position) && (m_angle == p.m_angle));
}

bool Pose::presqueEgales(const Pose& Pose) const
{
    return (m_position.presqueEgales(Pose.m_position)
            && AngleTools::anglesAlmostEqual(m_angle, Pose.m_angle));
}

Position Pose::getPosition() const
{
    return m_position;
}

void Pose::setPosition(const Position& p)
{
    m_position = p;
}

Angle Pose::getAngle() const
{
    return m_angle;
}

void Pose::setAngle(Angle a)
{
    m_angle = a;
}

void Pose::setX(Distance X)
{
    m_position.setX(X);
}

void Pose::setY(Distance Y)
{
    m_position.setY(Y);
}

#ifdef USE_ROS
Pose::operator geometry_msgs::Pose() const
{
    geometry_msgs::Pose p;
    p.position = m_position;
    p.orientation = tf::createQuaternionMsgFromYaw(m_angle);
    return p;
}

Pose::Pose(const geometry_msgs::Pose& p)
  : m_position(p.position)
{
    tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    m_angle = yaw;
}

Pose::Pose(const geometry_msgs::Transform& t)
  : m_position({t.translation.x,t.translation.y})
{
    tf::Quaternion q(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    m_angle = yaw;
}


#endif

#ifdef USE_IOSTREAM
std::ostream& operator<<(std::ostream& os, const Pose& p)
{
    os << "pose:{{p: " << p.getPosition() <<"}, angle: "<<p.getAngle()<<"rad}}";
}
#endif