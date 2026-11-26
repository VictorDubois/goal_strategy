#pragma once

#define NOMBRE_PIERRES_PAR_CARRIERE 3

/** @brief Orientation of a pierre (320 x 110 x 110 mm box) */
enum OrientationPierre
{
    horizontale,
    verticale,
};

class Pierre
{
public:
    Pierre(OrientationPierre a_orientation)
      : m_orientation(a_orientation) {};
    Pierre()
      : m_orientation(OrientationPierre::horizontale) {};

    OrientationPierre getOrientation() const
    {
        return m_orientation;
    };

private:
    OrientationPierre m_orientation;
};
