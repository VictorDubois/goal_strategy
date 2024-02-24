#pragma once

#include "krabilib/angle.h"
#include "krabilib/distance.h"
#include <Eigen/Dense>
#ifdef USE_IOSTREAM
#include <iostream>
#endif

#ifdef USE_ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#endif
typedef Eigen::Matrix3d Transform;
typedef Eigen::Affine3d Transform3D;
class PolarPosition;

/// @brief Classe permettant de stocker des positions en coordonnées cartésiennes
class Position
{
public:
    /// @brief Constructeur avec des coordonnées prédéfinies.
    Position(Distance X= Distance(0), Distance Y = Distance(0));

    /// @brief Constructeur avec des coordonnées prédéfinies.
    Position(const Eigen::Vector2d& position);

    /**
     * @brief Construct a new Position object using a PolarPosition
     * 
     * @param pp 
     */
    Position(const PolarPosition& pp);

    /// @brief Surchage d'opérateur pour multiplier par un flottant
    Position operator*(float val) const;

    /// @brief Surcharge d'opérateur pour assigner une position.
    void operator=(Position position);

    /// @brief Surchage d'opérateur pour ajouter et copier des coordonnées
    Position& operator+=(const Position& position);

    /// @brief Surchage d'opérateur pour soustraire et copier des coordonnées
    Position& operator-=(const Position& position);

    /// @brief Surchage d'opérateur pour soustraire et copier des coordonnées
    Position operator-(const Position& position);

    /// @brief Surchage d'opérateur pour multiplier les coordonées
    Position& operator*=(float val);

    /// @brief Surchage d'opérateur pour comparer des coordonnées
    bool operator==(const Position& p) const;

    /// @brief Fonction comparant l'égalité de positions à peu prés
    bool presqueEgales(const Position& p) const;

    /// @brief Fonction donnant la distance entre la position et le point de coordonnées nulles
    Distance getNorme(void) const;

    /// @brief Fonction donnant l'angle entre l'axe des absysses et la position
    Angle getAngle(void) const;

    /// @brief Pour récupérer X
    Distance getX() const;

    /// @brief Pour récupérer Y
    Distance getY() const;

    /// @brief Pour modifier X
    void setX(Distance X);

    /// @brief Pour modifier Y
    void setY(Distance Y);

    /**
     * @brief Transform the current point from the current frame to the frame defines by transform
     * 
     * @param t 3x3 2d transform matrix
     * @return Position new position in transformed frame
     */
    Position transform(const Transform& t);

    /**
     * @brief Transform the current point from the current frame to the frame defines by transform
     * 
     * @param t 3d transform matrix
     * @return Position new position in transformed frame
     */
    Position transform(const Transform3D& t);

#ifdef USE_ROS
    Position(const geometry_msgs::msg::Point& p);
    operator geometry_msgs::msg::Point() const;
#endif

#ifdef USE_IOSTREAM
    /**
     * @brief
     *
     * @param os output stream
     * @param p position
     * @return ostream&
     */
    friend std::ostream& operator<<(std::ostream& os, const Position& p);
#endif

private:
    // @brief Coordonnées
    Eigen::Vector2d m_pos;
};

class PolarPosition{
    public:
    PolarPosition(const Distance d = Distance(0),const Angle a = Angle(0));
    PolarPosition(const Position& pos);

    Distance getDistance() const;
    Angle getAngle() const;

#ifdef USE_IOSTREAM
    /**
     * @brief
     *
     * @param os output stream
     * @param p position
     * @return ostream&
     */
    friend std::ostream& operator<<(std::ostream& os, const PolarPosition& p);
#endif

    private:
    Distance m_dist;
    Angle m_angle;

};

#ifdef USE_ROS
    //Transform transformFromMsg(const geometry_msgs::msg::Transform& t);
    //Transform3D transform3DFromMsg(const geometry_msgs::msg::Transform& t);
#endif
