#include <math.h>

#include "Krabi/positionPlusAngle.h"
#ifdef USE_ROS
//#include "tf/tf.h"
//#include "geometry_msgs/msg/pose.hpp"
#endif

PositionPlusAngle::PositionPlusAngle()
{
}

PositionPlusAngle::PositionPlusAngle(const Position& pos, Angle ang)
    : position(pos), angle(ang)
{
}

PositionPlusAngle::PositionPlusAngle(const PositionPlusAngle& original)
    : position(original.position), angle(original.angle)
{
}

PositionPlusAngle PositionPlusAngle::getSymetrical()
{
    PositionPlusAngle pos(this->position.getSymetrical(), M_PI - angle);
    return pos;
}

PositionPlusAngle PositionPlusAngle::operator+(Distance distance) const
{
    return PositionPlusAngle(Position(position.getX() + distance*cos(angle), position.getY() + distance*sin(angle)), angle);
}

Vec3d PositionPlusAngle::operator+(const PositionPlusAngle& posAngAdd) const
{
    return Vec3d(position.getX() + posAngAdd.getPosition().getX(), position.getY() + posAngAdd.getPosition().getY(), angle + posAngAdd.getAngle());
}

PositionPlusAngle PositionPlusAngle::operator-(Distance distance) const
{
    return *this + (-distance);
}

Vec3d PositionPlusAngle::operator-(const PositionPlusAngle& posAngSub) const
{
    return Vec3d(position.getX() + posAngSub.getPosition().getX(), position.getY() + posAngSub.getPosition().getY(), angle + posAngSub.getAngle());
}

const PositionPlusAngle& PositionPlusAngle::operator=(const PositionPlusAngle& positionPlusAngle)
{
    position = positionPlusAngle.position;
    angle = positionPlusAngle.angle;

    return *this;
}

bool PositionPlusAngle::operator==(const PositionPlusAngle& p) const
{
    return ((position==p.position) && (angle==p.angle));
}

bool PositionPlusAngle::presqueEgales(const PositionPlusAngle& positionPlusAngle) const
{
    return(position.presqueEgales(positionPlusAngle.position) && AngleTools::anglesAlmostEqual(angle,positionPlusAngle.angle));
}

Position PositionPlusAngle::getPosition() const
{
    return position;
}

void PositionPlusAngle::setPosition(const Position& p)
{
    position = p;
}

Angle PositionPlusAngle::getAngle() const
{
    return angle;
}

void PositionPlusAngle::setAngle(Angle a)
{
    angle = a;
}

void PositionPlusAngle::setX(Distance X)
{
    position.setX(X);
}

void PositionPlusAngle::setY(Distance Y)
{
    position.setY(Y);
}

PositionPlusAngle PositionPlusAngle::operator+(const Vec3d &vec3d) const
{
    PositionPlusAngle resultat(Position(position.getX()+vec3d.x, position.getY()+vec3d.y), angle+vec3d.z);
    return resultat;
}

PositionPlusAngle PositionPlusAngle::operator-(const Vec3d &vec3d) const
{
    PositionPlusAngle resultat(Position(position.getX()-vec3d.x, position.getY()-vec3d.y), angle-vec3d.z);
    return resultat;
}

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = sin(0.5 * yaw);
  q.w = cos(0.5 * yaw);
  return q;
}

#ifdef USE_ROS
geometry_msgs::msg::Pose PositionPlusAngle::getPose() const
{
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Pose pose;
    point.x = position.getX();
    point.y = position.getY();
    point.z = 0;
    pose.position = point;
    pose.orientation = createQuaternionMsgFromYaw(angle);
    return pose;
}

PositionPlusAngle::PositionPlusAngle(const geometry_msgs::msg::Pose& pose, bool colorDependent)
{
    position = Position(pose.position, colorDependent);
    angle = 2*asin(pose.orientation.z);
}
#endif
