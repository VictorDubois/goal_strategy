#pragma once

#include "krabilib/angle.h"
#include "krabilib/position.h"

#ifdef USE_ROS
#include <geometry_msgs/Pose.h>

#endif

/// @brief Classe permettant d'avoir la position du robot par rapport à l'angle de la table et son
/// angle par rapport à l'absisse.
class Pose
{
public:
    /**@brief Constructeur de la classe.
     *@param Position pos est la position du robot au moment ou l'on déclare la classe
     *@param Angle ang est l'angle du robot au moment ou l'on déclare la classe.*/
    Pose(const Position& pos, Angle ang);

    Pose(const geometry_msgs::Transform& t);

    /// @brief Constructeur par défault de la classe.
    Pose();

    Pose getSymetrical();

    /// @brief Constructeur de copie.
    Pose(const Pose& original);

    /// @brief Surcharge d'opérateur pour ajouter une distance.
    Pose operator+(Distance distance) const;

    /// @brief Surcharge d'opérateur pour enlever une distance.
    Pose operator-(Distance distance) const;

    /// @brief Surcharge d'opérateur pour assigner un Pose
    const Pose& operator=(const Pose& Pose);

    /// @brief Surcharge d'opérateur pour comparer deux positions et angles.
    bool operator==(const Pose& p) const;

    /// @brief Fonction pour comparer deux Poses à peu prés.
    bool presqueEgales(const Pose& Pose) const;

    /// @brief Récupération de la position
    Position getPosition() const;

    /// @brief Envoi de la position
    void setPosition(const Position& p);

    /// @brief Récupération de l'angle
    Angle getAngle() const;

    /// @brief Pour modifier X
    void setX(Distance X);

    /// @brief Pour modifier Y
    void setY(Distance Y);

    /// @brief Envoi de l'angle
    void setAngle(Angle a);

#ifdef USE_ROS
    /// @brief Conversion vers geometry_msgs::Pose
    operator geometry_msgs::Pose() const;
    Pose(const geometry_msgs::Pose& p);
#endif

#ifdef USE_IOSTREAM
    friend std::ostream& operator<<(std::ostream& os, const Pose& p);
#endif

private:
    /// @brief position du robot par rapport à l'angle de la table
    Position m_position;

    /// @brief angle du robot par rapport au coté de la table
    Angle m_angle;
};