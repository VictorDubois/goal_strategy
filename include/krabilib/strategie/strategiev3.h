#pragma once

#ifndef STANDALONE_STRATEGIE
#include "krabilib/strategie/actionGoTo.h"
#endif

#include "krabilib/strategie/dijkstra.h"
#include "krabilib/strategie/mediumLevelAction.h"

//#define SMOOTH_MOVE

#ifdef QTGUI
#include <QColor>
#include <QPainter>
#endif

class StrategieV3
{

public:
    /** @brief Constructeur de la stratégie *
     * @param isYellow le côté de départ
     * @param useXSymetry est ce que la symetrie entre les deux couleur est selon l'axe X (true) ou
     * Y (false, comme en 2023) */
    StrategieV3(bool isYellow, bool useXSymetry = true);

    /** @brief Update la stratégie, soit parceque le robot est arrivé à une étape, soit parcequ'il
     * vient d'éviter un autre robot *
     * @return Le status  : 1 = vers une étape-objectif, 2 = vers une étape intermédiaire, -1 =
     * stratégie finie, plus rien à faire */
    virtual int update();

    /** @brief COnfigure la stratégie pour prendre en compte le fait qu'on vient de voir un robot */
    virtual void collisionAvoided();

    /** @brief Met à jour la trajectoire pour se diriger vers une étape intermédiaire */
    void updateIntermedaire();

    void resetEverything();

    void startDijkstra();

    /** @brief Retourne un pointeur sur l'étape en cours */
    Etape* getEtapeEnCours();

    /**
     * @brief Compute the absolute position based on if we are blue or yellow
     *
     * @param x_yellow x position if we were yellow
     * @param y_yellow y position if we were yellow
     * @return Position positiion in map frame
     */
    Position positionC(Distance x_yellow, Distance y_yellow);

    /**
     * @brief Compute the absolute position based on if we are blue or yellow
     *
     * @param x_yellow x position if we were yellow
     * @param y_yellow y position if we were yellow
     * @return Position positiion in map frame
     */
    Position positionC(double x_yellow, double y_yellow);

    bool isYellow();

    bool useXSymetry();

    double getRemainingTime();

    Etape* getGoal()
    {
        return m_tableau_etapes_total[m_goal];
    }

    void setRemainingTime(int64_t time_in_ms);

#ifdef QTGUI
    virtual void paint(QPainter* p);
#endif

protected:
    bool m_yellow;
    bool m_use_x_symetry;
    /** @brief le numéro de l'étape en cours */
    int m_etape_en_cours;
    int m_nombre_etapes;
    int m_numero_etape_garage;

    /** @brief tableau des actions qu'on peut décider de faire. TODO : mettre à jour ce tableau, en
     * incluant des actions plus diverses */
    /*MediumLevelAction** actionEtape;
    ActionGoTo* actionGoto;*/

    /** @brief la classe dijkstra pour calculer les distances */
    Dijkstra* m_dijkstra;

    /** @brief le tableau des étapes constituant le graphe à explorer */
    // std::vector<Etape*>& tableauEtapes;
    std::vector<Etape*>& m_tableau_etapes_total;

    /** @brief vient-on de détecter un robot */
    bool m_avoiding;

    /** @brief le numéro de l'étape-objectif */
    int m_goal;

    /** @brief le numéro de l'étape suivante */
    int m_next_step;

    /** @brief le status de la stratégie : 1 = vers une étape-objectif, 2 = vers une étape
     * intermédiaire, -1 = stratégie finie, plus rien à faire */
    int m_status_strat;

    /** @brief status du robot, est-il en train d'éviter en reculant */
    bool m_en_train_eviter_reculant;

    /** @brief status du robot, est-il en train d'éviter en avancant */
    bool m_en_train_eviter_avancant;

    void updateStock();
    virtual int getScoreEtape(int i) = 0;

#ifdef QTGUI
    QColor colorLiaisonsEtapes;
    QColor colorEtapeGoal;
    QColor colorEtapesIntermediaires;
    QColor colorEtapes;
    QColor colorTexteEtapes;
    QColor colorEtapesRobotVu;
#endif

private:
    bool updateScores();
    int64_t m_remaining_time_ms;
};
