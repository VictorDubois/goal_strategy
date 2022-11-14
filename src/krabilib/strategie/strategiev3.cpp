#include "krabilib/strategie/strategiev3.h"

#ifndef STANDALONE_STRATEGIE
#include "krabilib/asservissement.h"
#include "krabilib/leds.h"
#include "krabilib/odometrie.h"
#include "krabilib/strategie/actionGoTo.h"
#include "krabilib/strategieV2.h"
#endif

#include "krabilib/strategie/dijkstra.h"
#include "krabilib/strategie/etape.h"

#ifdef QTGUI
#include <QDebug>
#endif

StrategieV3::StrategieV3(bool isYellow, bool useXSymetry)
  : m_tableau_etapes_total(Etape::getTableauEtapesTotal())
{
    m_avoiding = false;
    m_etape_en_cours = 0;
    m_status_strat = 1;
    m_en_train_eviter_reculant = false;
    m_en_train_eviter_avancant = false;
    m_remaining_time_ms = 90 * 1000;
    m_yellow = isYellow;
    m_use_x_symetry = useXSymetry;

#ifdef QTGUI
    colorLiaisonsEtapes = QColor(150, 100, 50);
    colorEtapeGoal = QColor("red");
    colorEtapesIntermediaires = QColor("yellow");
    colorEtapes = QColor("orange");
    colorTexteEtapes = QColor("black");
    colorEtapesRobotVu = QColor("black");
#endif
}

Etape* StrategieV3::getEtapeEnCours()
{
    return m_tableau_etapes_total[m_etape_en_cours];
}

Position StrategieV3::positionC(Distance x_yellow, Distance y_yellow)
{
    if (isYellow())
    {
        return Position(Distance(-x_yellow), y_yellow);
    }

    if (useXSymetry())
    {
        return Position(Distance(x_yellow), y_yellow);
    }

    return Position(Distance(-x_yellow), Distance(-y_yellow));
}

Position StrategieV3::positionC(double x_yellow, double y_yellow)
{
    return positionC(Distance(x_yellow), Distance(y_yellow));
}

double StrategieV3::getRemainingTime()
{
    return m_remaining_time_ms / 1000.f;
}

void StrategieV3::setRemainingTime(int64_t time_in_ms)
{
    m_remaining_time_ms = time_in_ms;
}

bool StrategieV3::isYellow()
{
    return m_yellow;
}

bool StrategieV3::useXSymetry()
{
    return m_use_x_symetry;
}

int StrategieV3::update()
{
    // this->actionEtape[m_etape_en_cours]->reset();
    // this->actionGoto[m_etape_en_cours].reset();

    m_tableau_etapes_total[m_etape_en_cours]->reset();

    // Si on est en train d'éviter, on revient à l'étape précédente, et on marque l'étape comme
    // à éviter
    if (m_avoiding)
    {
#ifdef QTGUI
        qDebug() << "En train d'eviter";
#endif

        m_tableau_etapes_total[m_etape_en_cours]->robotVu();
        // m_tableau_etapes_total[m_etape_en_cours]->setState(-2);
        m_tableau_etapes_total[m_etape_en_cours]->getParent()->setParent(
          m_tableau_etapes_total[m_etape_en_cours]);
        m_etape_en_cours = m_tableau_etapes_total[m_etape_en_cours]->getParent()->getNumero();

        // On recalcul les distances par rapport à l'étape où l'on vient d'arriver
        m_dijkstra->setEtapeCourante(m_etape_en_cours);

        if (m_en_train_eviter_reculant)
        {
            m_en_train_eviter_reculant = false;
            m_en_train_eviter_avancant = true;

#ifndef STANDALONE_STRATEGIE
            m_tableau_etapes_total[etapeEnCours]->getActionGoTo()->setGoBack(false);
#else
// @TODO setGoBack to false
#endif // STANDALONE_STRATEGIE
       // actionEtape[etapeEnCours]->setGoBack(false);
        }
        else
        {
            m_en_train_eviter_reculant = true;
            m_en_train_eviter_avancant = false;
#ifndef STANDALONE_STRATEGIE
            m_tableau_etapes_total[etapeEnCours]->getActionGoTo()->setGoBack(true);
#else
// @TODO setGoBack to true
#endif // STANDALONE_STRATEGIE
       // actionEtape[etapeEnCours]->setGoBack(true);
        }

#ifndef STANDALONE_STRATEGIE
        StrategieV2::addTemporaryAction(m_tableau_etapes_total[etapeEnCours]->getActionGoTo());
#else
        // @TODO addTempAction
#endif // STANDALONE_STRATEGIE
       // StrategieV2::addTemporaryAction(actionEtape[etapeEnCours]);
       // m_dijkstra->setEtapeCourante((m_tableau_etapes_total[m_etape_en_cours]->getParent()->getNumero()));
        if (m_dijkstra->run() != 0)
        {
            // Si run renvoit autre chose que 0, c'est que l'étape en cours a changée.
            // Cela arrive pour débloquer le robot
            // Etape* ancienneEtape = m_tableau_etapes_total[m_etape_en_cours];
            // m_etape_en_cours =
            // m_tableau_etapes_total[m_etape_en_cours]->getParent()->getNumero();
            /*this->actionEtape[m_etape_en_cours]->reset();
            this->actionGoto[m_etape_en_cours].reset();*/
            m_tableau_etapes_total[m_etape_en_cours]->reset();
        }

        // On retourne à l'étape intermédiaire précédente, en marche arrière

        m_avoiding = false;
        m_status_strat = 1;
    }
    else
    {
#ifdef QTGUI
        qDebug() << "Pas en train d'eviter";
#endif

        // On reset toute les directions à aller en marche avant
        for (int i = 0; i < m_nombre_etapes; i++)
        {
            /*actionGoto[i].setGoBack(false);
            actionEtape[i]->setGoBack(false);*/
            m_tableau_etapes_total[i]->setGoBack(false);
        }
        m_en_train_eviter_reculant = false;
        m_en_train_eviter_avancant = false;

        if (m_status_strat == 2) // Si on vient d'arriver à une étape intermédiare
        {
            this->updateIntermedaire();
        }
        else // Sinon, statusStrat==1, et il faut donc choisir un nouvel objectif
        {
            // Si on n'était pas en train d'éviter
            if (!(m_en_train_eviter_reculant || m_en_train_eviter_avancant))
            {
                // L'objectif qu'on vient de remplir est maintenant un simple point de passage
                // m_tableau_etapes_total[m_etape_en_cours]->finir();//Vieux, on utilise
                // maintenant updateStock dans la strat de l'année en cours Idem pour les autres
                // étapes correspondant au même objectif
                for (int etapeLiee = 0; etapeLiee < m_tableau_etapes_total[m_etape_en_cours]
                                                      ->getNombreEtapesLieesParFinirEtape();
                     etapeLiee++)
                {
                    int numeroEtapeLiee = m_tableau_etapes_total[m_etape_en_cours]
                                            ->getEtapesLieesParFinirEtape()[etapeLiee];
                    m_tableau_etapes_total[numeroEtapeLiee]->finir();
                }

                // Mise à jour du stock et l'objectif qu'on vient de remplir est maintenant un
                // simple point de passage
                this->updateStock();

                // On est maintenant arrivé à l'étape de fin de l'action (en général c'est la
                // même étape, mais pas toujours, ex : les claps de 2015)
                m_etape_en_cours
                  = m_tableau_etapes_total[m_etape_en_cours]->getNumeroEtapeFinAction() == -1
                      ? m_etape_en_cours
                      : m_tableau_etapes_total[m_etape_en_cours]->getNumeroEtapeFinAction();
            }

            int score = 0;
            bool resteDesChosesAFaire = updateScores();

            // S'il n'y a plus d'objectif dans tout le graphe, on se replit vers une position où
            // on ne bloque pas l'adversaire. Sinon, il y a risque de prendre un avertissement
            // pour anti-jeu (évité de peu pour le premier match de Krabi 2014)
            if (!resteDesChosesAFaire)
            {
                for (int i = 0; i < m_nombre_etapes; i++)
                {
                    m_tableau_etapes_total[i]->oublieRobotVu();
                }
                resteDesChosesAFaire = updateScores();

                // S'il n'y a VRAIMENT plus rien à faire
                if (!resteDesChosesAFaire)
                {
                    // Si on est au garage, on s'arrête
                    if (m_etape_en_cours == m_numero_etape_garage)
                    {
                        m_status_strat = -1; // Plus rien à faire, on passe à l'action suivante
                                             // de stratégieV2
#ifdef QTGUI
                        qDebug() << "Fin de StrategieV3";
#endif
                        return m_status_strat;
                    }
                    else
                    {
                        // Sinon on y va
                        m_tableau_etapes_total[m_numero_etape_garage]->setScore(1000);
                    }
                }
            }

            // On recalcul les distances par rapport à l'étape où l'on vient d'arriver
            m_dijkstra->setEtapeCourante(m_etape_en_cours);

            if (m_dijkstra->run() != 0)
            {
                // Si run renvoit autre chose que 0, c'est que l'étape en cours a changée.
                // Cela arrive pour débloquer le robot
                // Etape* ancienneEtape = m_tableau_etapes_total[m_etape_en_cours];
                // m_etape_en_cours =
                // m_tableau_etapes_total[m_etape_en_cours]->getParent()->getNumero();
                m_tableau_etapes_total[m_etape_en_cours]->getAction()->reset();
            }

            // On sélectionne l'objectif le plus prometteur : pas trop loin et qui rapporte
            int meilleurEtape = -1;
            int scoreMaxi = -100000;

            int scoreTypeEtape = 0;
            for (int i = 0; i < m_nombre_etapes; i++)
            {
                scoreTypeEtape = m_tableau_etapes_total[i]->getScore();
                //        score =
                //        modificateurTemporel*(10000-m_tableau_etapes_total[i]->getDistance() +
                //        scoreTypeEtape);
                score = (10000 - m_tableau_etapes_total[i]->getDistance() + scoreTypeEtape);
                if ((scoreMaxi < score) && scoreTypeEtape
                    && (m_tableau_etapes_total[i]->getDistance() != -1))
                {
                    scoreMaxi = score;
                    meilleurEtape = i;
                }
            }

            if (meilleurEtape == -1)
            {
                if (m_etape_en_cours == m_numero_etape_garage)
                {
                    m_status_strat
                      = -1; // Plus rien à faire, on passe à l'action suivante de stratégieV2
                    return m_status_strat;
                }
                else
                {
                    meilleurEtape = m_numero_etape_garage;
                }
            }

            m_goal = meilleurEtape;
            m_status_strat = 2; // Jusqu'à preuve du contraire, la prochaine étape est une étape
                                // intermédiaire
            this->updateIntermedaire(); // On y va
        }
    }

    return m_status_strat;
}

void StrategieV3::resetEverything()
{
    for (int i = 0; i < 10; i++)
    {
        m_tableau_etapes_total[i]->setState(0);
    }
}

void StrategieV3::collisionAvoided()
{
    m_avoiding = true;
}

void StrategieV3::updateIntermedaire()
{
    // Note : le parent d'une étape est l'étape le rapprochant le plus de l'étape d'origine
    // Ainsi, le parent du parent du parent... de n'importe quelle étape est l'étape d'origine
    //(sauf peut être le parent de l'étape d'origine, mais on s'en fout

#ifdef QTGUI
    qDebug() << "updateIntermedaire\n";
#endif
    int etapeOuOnVientDArriver = m_etape_en_cours;
    m_etape_en_cours = m_goal;
    m_next_step = -1;

    // Si la prochaine étape est le goal, alors au prochain update il faudra trouver un nouvel
    // objectif -> status = 1;
    if (((m_tableau_etapes_total[m_etape_en_cours]->getParent()->getNumero()))
        == etapeOuOnVientDArriver)
    {
#ifdef QTGUI
        qDebug() << "la prochaine etape est le goal\n" << etapeOuOnVientDArriver;
#endif
        m_status_strat = 1;
    }

    // On cherche l'etape suivant vers l'etape - but
    while (((m_tableau_etapes_total[m_etape_en_cours]->getParent()->getNumero()))
           != etapeOuOnVientDArriver)
    {

        m_next_step = m_etape_en_cours;
        m_etape_en_cours = ((m_tableau_etapes_total[m_etape_en_cours]->getParent()->getNumero()));
    }

    if (m_status_strat == 1)
    {
// On réalise l'action de l'étape - but
#ifndef STANDALONE_STRATEGIE
        StrategieV2::addTemporaryAction(m_tableau_etapes_total[m_etape_en_cours]->getAction());
#else
        // @TODO add temp action
#endif // STANDALONE_STRATEGIE
    }
    else
    {
        // On ajoute l'action d'aller en ligne droite vers cette étape intermédiaire
#ifdef SMOOTH_MOVE
        m_tableau_etapes_total[m_etape_en_cours]->getActionGoTo()->setNextGoal(
          m_tableau_etapes_total[m_next_step]->getPosition());
#endif

#ifndef STANDALONE_STRATEGIE
        StrategieV2::addTemporaryAction(m_tableau_etapes_total[m_etape_en_cours]->getActionGoTo());
#else
        // @TODO add temp action
#endif // STANDALONE_STRATEGIE
    }
}

#ifdef QTGUI
void StrategieV3::paint(QPainter* p)
{
    for (int numeroEtape = 0; numeroEtape < m_nombre_etapes; numeroEtape++)
    {
        QPoint position = QPoint(m_tableau_etapes_total[numeroEtape]->getPosition().getX(),
                                 m_tableau_etapes_total[numeroEtape]->getPosition().getY());

        // Affichage des étapes
        p->setPen(colorEtapesIntermediaires);   //"orange"));
        p->setBrush(colorEtapesIntermediaires); //"orange"));
        p->setOpacity(1.0f);
        p->drawEllipse(position, 10, 10);

        // Etape - but en surbrillance
        if (numeroEtape == m_goal)
        {
            p->setPen(colorEtapeGoal);
            p->setBrush(colorEtapeGoal);
            p->setOpacity(1.0f);
            p->drawEllipse(position, 30, 30);
        }

        // Etape actuelle en surbrillance
        if (numeroEtape == m_etape_en_cours)
        {
            p->setPen(colorEtapesIntermediaires);
            p->setBrush(colorEtapesIntermediaires);
            p->setOpacity(1.0f);
            p->drawEllipse(position, 25, 25);
        }

        // Etapes où on a vu un robot
        if (m_tableau_etapes_total[numeroEtape]->aEviter())
        {
            p->setPen(colorEtapesRobotVu);
            p->setBrush(colorEtapesRobotVu);
            p->setOpacity(1.0f);
            p->drawEllipse(position, 20, 20);
        }

        // Affichage du numéro des étapes
        QFont font;
        font.setPixelSize(50);
        p->setFont(font);
        p->setOpacity(1);
        p->setPen(colorTexteEtapes);
        p->setBrush(colorTexteEtapes);
        p->drawText(position, QString::number(m_tableau_etapes_total[numeroEtape]->getNumero()));

        // Affichage des liaisons entre étapes
        for (int numeroChild = 0;
             numeroChild < m_tableau_etapes_total[numeroEtape]->getNbChildren();
             numeroChild++)
        {
            //            Affichages de liaisons
            //            QPoint positionChild = QPoint(
            //            m_tableau_etapes_total[numeroEtape]->getChildren()[numeroChild]->getPosition().getX(),
            //            -(m_tableau_etapes_total[numeroEtape]->getChildren()[numeroChild]->getPosition().getY()));

            // Affichage des demi-liaisons, pour mieux voir les liens mono-directionnels
            QPoint positionChild = QPoint(
              (m_tableau_etapes_total[numeroEtape]->getChildren()[numeroChild]->getPosition().getX()
               + m_tableau_etapes_total[numeroEtape]->getPosition().getX())
                / 2,
              (m_tableau_etapes_total[numeroEtape]->getChildren()[numeroChild]->getPosition().getY()
               + m_tableau_etapes_total[numeroEtape]->getPosition().getY())
                / 2);
            p->setOpacity(0.5f);
            p->setPen(colorLiaisonsEtapes);
            p->setBrush(colorLiaisonsEtapes);
            p->drawLine(position.x(), position.y(), positionChild.x(), positionChild.y());
            p->drawEllipse(position, 10, 10);
        }

        // Affichage du type d'étape
        QPoint positionTypeEtape
          = QPoint(m_tableau_etapes_total[numeroEtape]->getPosition().getX(),
                   m_tableau_etapes_total[numeroEtape]->getPosition().getY() + 50);
        p->setFont(font);
        p->setOpacity(0.5);
        p->setPen(colorTexteEtapes);
        p->setBrush(colorTexteEtapes);
        p->drawText(positionTypeEtape,
                    Etape::getShortNameType(m_tableau_etapes_total[numeroEtape]->getEtapeType()));
    }
    p->setOpacity(1);
}
#endif

void StrategieV3::startDijkstra()
{

    for (int i = 0; i < m_nombre_etapes; i++)
        m_tableau_etapes_total[i]->computeChildDistances();

    this->m_dijkstra = new Dijkstra(m_tableau_etapes_total, m_nombre_etapes);

    m_tableau_etapes_total[0]->setParent(
      m_tableau_etapes_total[0]); // Evite de planter si on detecte dès la première boucle (dans
                                  // le simu)

    m_dijkstra->setEtapeCourante(0);
}

void StrategieV3::updateStock()
{
    switch (m_tableau_etapes_total[m_etape_en_cours]->getEtapeType())
    {
    case Etape::DEPART:
        m_tableau_etapes_total[m_etape_en_cours]->setEtapeType(Etape::POINT_PASSAGE);
        break;
    case Etape::PRE_BAFFE:
        m_tableau_etapes_total[m_etape_en_cours]->setEtapeType(Etape::POINT_PASSAGE);
        break;
    case Etape::ASSIETTE:
        // Une assiette reste une assiette
        // m_tableau_etapes_total[m_etape_en_cours]->setEtapeType(Etape::POINT_PASSAGE);
        break;
    default:
        m_tableau_etapes_total[m_etape_en_cours]->setEtapeType(Etape::POINT_PASSAGE);
        break;
    }
}

bool StrategieV3::updateScores()
{
    // Mise à jour du score correspondant à chaque étape
    // CETTE PARTIE EST A AMELIORER
    // on pourrait prendre en compte :
    // le fait le transporter un feu, ce qui active les objectifs de dépose done
    // le temps que prend chaque action
    // le temps restant done
    // la trajectoire de notre autre robot
    //...
    // C'est aussi utilisé pour savoir si on est dans un sous-graphe sans objectif,
    // et qu'il faut oublier qu'on a vu des robots
    int scoreTypeEtape = 0;
    // float modificateurTemporel = 1.f;

    bool resteDesChosesAFaire = false;
    for (int i = 0; i < m_nombre_etapes; i++)
    {
        scoreTypeEtape = this->getScoreEtape(i);
        if (scoreTypeEtape)
        {
            resteDesChosesAFaire = true;
        }
        this->m_tableau_etapes_total[i]->setScore(scoreTypeEtape);
    }
    return resteDesChosesAFaire;
}
