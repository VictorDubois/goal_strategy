#ifndef POSITION_H
#define POSITION_H

#include "Krabi/angle.h"
#include "Krabi/distance.h"
#include "Krabi/vec2d.h"

#ifdef USE_IOSTREAM
#include <iostream>
#include "geometry_msgs/Point.h"
#endif // USE_IOSTREAM

class StrategieV2;

#define COLOR_POSITION Position::colorPosition

/// @brief Classe permettant de stocker des positions en coordonnées cartésiennes
class Position
{
    public:

        /// @brief Constructeur par défaut avec des coordonnées nulles.
        Position();

        /// @brief Constructeur avec des coordonnées prédéfinies.
        Position(Distance X, Distance Y, bool colorDependent = false);

        Position getSymetrical();

        /// @brief Surchage d'opérateur pour ajouter des coordonnées
        Vec2d operator+(const Position &position) const;

        /// @brief Surchage d'opérateur pour soustraire des coordonnées
        Vec2d operator-(const Position &position) const;

        /// @brief Surchage d'opérateur pour multiplier par un flottant
        Position operator*(float val) const;

        static Position colorPosition(Distance x, Distance y, bool isYellow = false);

        /// @brief Surcharge d'opérateur pour assigner une position.
        void operator=(Position position);

        /// @brief Surchage d'opérateur pour ajouter et copier des coordonnées
        Position operator+=(const Position &position);

        /// @brief Surchage d'opérateur pour soustraire et copier des coordonnées
        Position operator-=(const Position &position);

        /// @brief Surchage d'opérateur pour multiplier les coordonées
        bool operator*=(float val);

        /// @brief Surchage d'opérateur pour comparer des coordonnées
        bool operator==(const Position &p) const;

        /// @brief Surchage d'opérateur pour ajouter des coordonnées
        Position operator+(const Vec2d &vec2d) const;

        /// @brief Surchage d'opérateur pour soustraire des coordonnées
        Position operator-(const Vec2d &vec2d) const;

        /// @brief Fonction comparant l'égalité de positions à peu prés
        bool presqueEgales(const Position &p) const;

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


        #ifdef USE_IOSTREAM
        /// @brief Decrit la position
        std::string Print();
        #endif // USE_IOSTREAM

        /// @brief Coordonnées
        Distance x;
        Distance y;

	#ifdef USE_ROS
	/// @brief Conversion vers geometry_msgs::Point
	geometry_msgs::Point getPoint() const;

        /// @brief Constructeur depuis geometry_msgs::Point
        Position(const geometry_msgs::Point&, bool colorDependent);
	#endif
};

#endif
