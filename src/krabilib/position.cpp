#include "krabilib/position.h"
#include <math.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Constructeur par défaut avec des coordonnées nulles.
Position::Position(Distance x, Distance y)
  : m_pos(x, y)
{
}

Position::Position(const Eigen::Vector2d& position)
  : m_pos(position)
{
}

Position::Position(const PolarPosition& pp)
{
    m_pos[0] = pp.getDistance() * cos(pp.getAngle());
    m_pos[1] = pp.getDistance() * sin(pp.getAngle());
}

Distance Position::getX() const
{
    return Distance(m_pos.x());
}

Distance Position::getY() const
{
    return Distance(m_pos.y());
}

void Position::setX(Distance X)
{
    m_pos.x() = X;
}

void Position::setY(Distance Y)
{
    m_pos.y() = Y;
}

/// @brief Surchage d'opérateur pour multiplier par un flottant
Position Position::operator*(float val) const
{
    Position resultat(m_pos * val);
    return resultat;
}

void Position::operator=(Position position)
{

    m_pos.x() = position.m_pos.x();
    m_pos.y() = position.m_pos.y();
}

Position& Position::operator+=(const Position& position)
{
    m_pos.x() += position.m_pos.x();
    m_pos.y() += position.m_pos.y();

    return *this;
}

Position& Position::operator-=(const Position& position)
{
    m_pos.x() -= position.m_pos.x();
    m_pos.y() -= position.m_pos.y();

    return *this;
}

bool Position::presqueEgales(const Position& p) const
{
    return (DistanceTools::distancePresqueEgales(getX(), p.getX())
            && DistanceTools::distancePresqueEgales(getY(), p.getY()));
}

bool Position::operator==(const Position& p) const
{
    return (m_pos.x() == p.m_pos.x() && m_pos.y() == p.m_pos.y());
}

Position& Position::operator*=(float val)
{
    m_pos *= val;
    return *this;
}

Position Position::operator-(const Position& p)
{
    return Position(m_pos - p.m_pos);
}

Distance Position::getNorme() const
{
    return Distance(m_pos.norm());
}

Angle Position::getAngle() const
{
    return Angle(atan2(m_pos.y(), m_pos.x()));
}

#ifdef USE_ROS
Position::Position(const geometry_msgs::msg::Point& p)
{
    m_pos << p.x, p.y;
}

Position::operator geometry_msgs::msg::Point() const
{
    geometry_msgs::msg::Point pt;
    pt.x = m_pos.x();
    pt.y = m_pos.y();
    pt.z = 0;
    return pt;
}

Transform transformFromMsg(const geometry_msgs::msg::Transform& t)
{
    Transform out;
    tf2::Quaternion q(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    out << cos(yaw), -sin(yaw), t.translation.x, sin(yaw), cos(yaw), t.translation.y, 0, 0, 1;

    return out;
}

Transform3D transform3DFromMsg(const geometry_msgs::msg::Transform& t)
{
    tf2::Transform tmp;
    tf2::fromMsg(t, tmp);
    Eigen::Affine3d out;
    for (int i = 0; i < 3; i++)
    {
        out.matrix()(i, 3) = tmp.getOrigin()[i];
        for (int j = 0; j < 3; j++)
        {
            out.matrix()(i, j) = tmp.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        out.matrix()(3, col) = 0;
    out.matrix()(3, 3) = 1;

    return out;
}

#endif

#ifdef USE_IOSTREAM
std::ostream& operator<<(std::ostream& os, const Position& p)
{
    os << "p:{x:" << double(p.getX()) << "m, y: " << double(p.getY()) << "m}";
}

std::ostream& operator<<(std::ostream& os, const PolarPosition& p)
{
    os << "p:{r: " << double(p.getDistance()) << "m, theta: " << double(p.getAngle()) << "rad}";
}
#endif

PolarPosition::PolarPosition(const Distance d, const Angle a)
  : m_dist(d)
  , m_angle(a)
{
}
PolarPosition::PolarPosition(const Position& pos)
{
    m_angle = pos.getAngle();
    m_dist = pos.getNorme();
}

Distance PolarPosition::getDistance() const
{
    return m_dist;
};
Angle PolarPosition::getAngle() const
{
    return m_angle;
};

Position Position::transform(const Transform& t)
{
    return Position((t * m_pos.homogeneous()).head<2>());
}

Position Position::transform(const Transform3D& t)
{
    Eigen::Vector3d tmp;
    tmp << m_pos, 0;
    return Position((t * tmp).head<2>());
}
