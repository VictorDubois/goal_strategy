#pragma once

#include <chrono>
#include <mutex>

/**
 * Décide quand la tirette autorise le départ du match.
 *
 * Le robot peut démarrer SANS tirette. Le match ne part donc que sur la séquence complète :
 * insérée de façon stable (armée), puis retirée. Le temps de stabilité évite qu'un rebond du
 * contact à l'insertion (insérée / retirée / insérée) lance le match.
 *
 * Partagé par Core (main_strategy) et GoalStrat (goal_strategy) pour qu'ils décident pareil.
 * Thread-safe : les callbacks et loop() peuvent tourner sur des threads différents.
 */
class TiretteFilter
{
public:
    explicit TiretteFilter(double a_stable_s = 1.0)
      : m_stable_s(a_stable_s)
    {
    }

    /// Horloge monotone : insensible au temps simulé (qui saute au démarrage de /clock)
    static double steadyNow()
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
          .count();
    }

    /**
     * @param a_removed valeur du topic tirette (true = retirée / absente)
     * @return true une seule fois : à l'instant où le match doit démarrer
     */
    bool update(bool a_removed, double a_now = steadyNow())
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!a_removed)
        {
            if (!m_inserted)
            {
                m_inserted = true;
                m_inserted_since = a_now;
            }
            if (a_now - m_inserted_since >= m_stable_s)
            {
                m_armed = true;
            }
            return false;
        }

        m_inserted = false;
        if (m_armed && !m_started)
        {
            m_started = true;
            return true;
        }
        return false;
    }

    /// La tirette a été vue insérée de façon stable
    bool armed() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_armed;
    }

    /// Le match a été lancé (armée puis retirée)
    bool started() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_started;
    }

private:
    const double m_stable_s;
    mutable std::mutex m_mutex;
    bool m_inserted = false;
    double m_inserted_since = 0.;
    bool m_armed = false;
    bool m_started = false;
};
