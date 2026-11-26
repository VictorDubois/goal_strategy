#pragma once

#include "krabilib/strategie/pierre.h"
#include <vector>

/** @brief Number of actuators able to hold pierres. TODO: not decided yet (2 or 3). */
#define NOMBRE_ACTIONNEURS 3

#define NOMBRE_PIERRES_MAX_PAR_ACTIONNEUR 3

/** @brief One actuator of the main robot and the pierres it holds (0 to 3) */
class Actionneur
{
public:
    /** @param a_peut_construire_porte_tour only some actuators can build a porte or a tour; all can build a mur */
    Actionneur(bool a_peut_construire_porte_tour = true)
      : m_peut_construire_porte_tour(a_peut_construire_porte_tour) {};

    int getNombrePierres() const
    {
        return static_cast<int>(m_pierres.size());
    };
    bool estVide() const
    {
        return m_pierres.empty();
    };
    bool estPlein() const
    {
        return getNombrePierres() >= NOMBRE_PIERRES_MAX_PAR_ACTIONNEUR;
    };

    void ajoutePierres(const std::vector<Pierre>& a_pierres)
    {
        for (const auto& l_pierre : a_pierres)
        {
            if (!estPlein())
            {
                m_pierres.push_back(l_pierre);
            }
        }
    };
    void vide()
    {
        m_pierres.clear();
    };

    bool peutConstruireMur() const
    {
        return estPlein();
    };
    bool peutConstruirePorte() const
    {
        return m_peut_construire_porte_tour && estPlein();
    };
    bool peutConstruireTour() const
    {
        return m_peut_construire_porte_tour && !estVide();
    };

private:
    bool m_peut_construire_porte_tour;
    std::vector<Pierre> m_pierres;
};

/** @brief Pierres carried by the main robot, per actuator. Only the porte is tracked as
 * "already built": other used spots become regular waypoints (POINT_PASSAGE) once done. */
class StockPierres
{
public:
    StockPierres()
    {
        // TODO: decide which actuators can build a porte or a tour
        for (int i = 0; i < NOMBRE_ACTIONNEURS; i++)
        {
            m_actionneurs.emplace_back(true);
        }
    };

    std::vector<Actionneur>& getActionneurs()
    {
        return m_actionneurs;
    };

    int getNombrePierres() const
    {
        int l_total = 0;
        for (const auto& l_actionneur : m_actionneurs)
        {
            l_total += l_actionneur.getNombrePierres();
        }
        return l_total;
    };

    bool peutRamasser() const
    {
        for (const auto& l_actionneur : m_actionneurs)
        {
            if (!l_actionneur.estPlein())
            {
                return true;
            }
        }
        return false;
    };
    bool peutConstruireMur() const
    {
        for (const auto& l_actionneur : m_actionneurs)
        {
            if (l_actionneur.peutConstruireMur())
            {
                return true;
            }
        }
        return false;
    };
    bool peutConstruireTour() const
    {
        for (const auto& l_actionneur : m_actionneurs)
        {
            if (l_actionneur.peutConstruireTour())
            {
                return true;
            }
        }
        return false;
    };
    /** @brief At most one porte per team */
    bool peutConstruirePorte() const
    {
        if (m_porte_construite)
        {
            return false;
        }
        for (const auto& l_actionneur : m_actionneurs)
        {
            if (l_actionneur.peutConstruirePorte())
            {
                return true;
            }
        }
        return false;
    };

    bool isPorteConstruite() const
    {
        return m_porte_construite;
    };
    void setPorteConstruite(bool a_porte_construite)
    {
        m_porte_construite = a_porte_construite;
    };

private:
    std::vector<Actionneur> m_actionneurs;
    bool m_porte_construite = false;
};
