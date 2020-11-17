#pragma once

#ifndef STANDALONE_STRATEGIE
#include "krabilib/strategie/actionGoTo.h"
#endif

#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/dijkstra.h"

//#define SMOOTH_MOVE

#ifdef QTGUI
#include <QPainter>
#include <QColor>
#endif

/*#ifdef KRABI_JR

    #define NOMBRE_ETAPES 31//22//10

#endif*/
//#define ETAPE_GARAGE 1
//#define NOMBRE_ETAPES 10
class StrategieV3{

public:
    static bool isYellow();
    /** @brief Constructeur de la stratégie *
    * @param isYellow le côté de départ */
    StrategieV3(bool isYellow);

    /** @brief Update la stratégie, soit parceque le robot est arrivé à une étape, soit parcequ'il vient d'éviter un autre robot *
    * @return Le status  : 1 = vers une étape-objectif, 2 = vers une étape intermédiaire, -1 = stratégie finie, plus rien à faire */
    virtual int update();

    /** @brief COnfigure la stratégie pour prendre en compte le fait qu'on vient de voir un robot */
    virtual void collisionAvoided();

    /** @brief Met à jour la trajectoire pour se diriger vers une étape intermédiaire */
    void updateIntermedaire();

    void resetEverything();

    void startDijkstra();

    /** @brief Retourne un pointeur sur l'étape en cours */
    Etape* getEtapeEnCours();

#ifdef QTGUI
    virtual void paint(QPainter* p);
#endif

protected:
    static bool yellow;
    /** @brief le numéro de l'étape en cours */
    int etapeEnCours;
    int nombreEtapes;
    int numeroEtapeGarage;

    /** @brief tableau des actions qu'on peut décider de faire. TODO : mettre à jour ce tableau, en incluant des actions plus diverses */
    /*MediumLevelAction** actionEtape;
    ActionGoTo* actionGoto;*/

    /** @brief la classe dijkstra pour calculer les distances */
    Dijkstra* dijkstra;

    /** @brief le tableau des étapes constituant le graphe à explorer */
    //Etape** tableauEtapes;
    Etape** tableauEtapesTotal;

    /** @brief vient-on de détecter un robot */
    bool avoiding;

    /** @brief le numéro de l'étape-objectif */
    int goal;

    /** @brief le numéro de l'étape suivante */
    int nextStep;

    /** @brief le status de la stratégie : 1 = vers une étape-objectif, 2 = vers une étape intermédiaire, -1 = stratégie finie, plus rien à faire */
    int statusStrat;

    /** @brief status du robot, est-il en train d'éviter en reculant */
    bool enTrainEviterReculant;

    /** @brief status du robot, est-il en train d'éviter en avancant */
    bool enTrainEviterAvancant;

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
    int64_t millisecondesRestantes;
};