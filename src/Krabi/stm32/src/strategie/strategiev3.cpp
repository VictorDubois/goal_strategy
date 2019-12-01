#include "Krabi/strategie/strategiev3.h"

#ifndef STANDALONE_STRATEGIE
	#include "Krabi/strategieV2.h"
	#include "Krabi/strategie/actionGoTo.h"
	#include "Krabi/leds.h"
	#include "Krabi/odometrie.h"
	#include "Krabi/asservissement.h"
#endif

#include "Krabi/strategie/dijkstra.h"
#include "Krabi/strategie/etape.h"

#ifdef QTGUI
    #include <QDebug>
#endif
//#include <iostream>

StrategieV3::StrategieV3(bool /*isYellow*/) : MediumLevelAction()
{
    this->avoiding = false;
    this->etapeEnCours = 0;
    this->statusStrat=1;
    this->enTrainEviterReculant = false;
    this->enTrainEviterAvancant = false;
    this->millisecondesRestantes = 90 * 1000;

#ifdef QTGUI
    colorLiaisonsEtapes = QColor(150,100,50);
    colorEtapeGoal = QColor("red");
    colorEtapesIntermediaires = QColor("yellow");
    colorEtapes = QColor("orange");
    colorTexteEtapes = QColor("black");
    colorEtapesRobotVu = QColor("black");
#endif
}

Etape* StrategieV3::getEtapeEnCours()
{
    return this->tableauEtapesTotal[this->etapeEnCours];
}

int StrategieV3::update()
{
    //this->actionEtape[this->etapeEnCours]->reset();
    //this->actionGoto[this->etapeEnCours].reset();


    tableauEtapesTotal[this->etapeEnCours]->reset();

    //Si on est en train d'éviter, on revient à l'étape précédente, et on marque l'étape comme à éviter
    if(this->avoiding)
    {
        #ifdef QTGUI
        qDebug() << "En train d'eviter";
        #endif

        this->tableauEtapesTotal[this->etapeEnCours]->robotVu();
        //this->tableauEtapesTotal[this->etapeEnCours]->setState(-2);
        this->tableauEtapesTotal[etapeEnCours]->getParent()->setParent(this->tableauEtapesTotal[this->etapeEnCours]);
        this->etapeEnCours = this->tableauEtapesTotal[etapeEnCours]->getParent()->getNumero();

        //On recalcul les distances par rapport à l'étape où l'on vient d'arriver
        dijkstra->setEtapeCourante(this->etapeEnCours);

        if(this->enTrainEviterReculant)
        {
            this->enTrainEviterReculant = false;
            this->enTrainEviterAvancant = true;

            #ifndef STANDALONE_STRATEGIE
            tableauEtapesTotal[etapeEnCours]->getActionGoTo()->setGoBack(false);
            #else
            // @TODO setGoBack to false
            #endif // STANDALONE_STRATEGIE
            //actionEtape[etapeEnCours]->setGoBack(false);
        }
        else
        {
            this->enTrainEviterReculant = true;
            this->enTrainEviterAvancant = false;
            #ifndef STANDALONE_STRATEGIE
            tableauEtapesTotal[etapeEnCours]->getActionGoTo()->setGoBack(true);
            #else
            // @TODO setGoBack to true
            #endif // STANDALONE_STRATEGIE
            //actionEtape[etapeEnCours]->setGoBack(true);
        }

        #ifndef STANDALONE_STRATEGIE
            StrategieV2::addTemporaryAction(tableauEtapesTotal[etapeEnCours]->getActionGoTo());
        #else
            // @TODO addTempAction
        #endif // STANDALONE_STRATEGIE
        //StrategieV2::addTemporaryAction(actionEtape[etapeEnCours]);
        //dijkstra->setEtapeCourante((this->tableauEtapesTotal[this->etapeEnCours]->getParent()->getNumero()));
        if(dijkstra->run() != 0)
        {
            // Si run renvoit autre chose que 0, c'est que l'étape en cours a changée.
            // Cela arrive pour débloquer le robot
            //Etape* ancienneEtape = this->tableauEtapesTotal[this->etapeEnCours];
            //this->etapeEnCours = this->tableauEtapesTotal[this->etapeEnCours]->getParent()->getNumero();
            /*this->actionEtape[this->etapeEnCours]->reset();
            this->actionGoto[this->etapeEnCours].reset();*/
            tableauEtapesTotal[this->etapeEnCours]->reset();
        }

        //On retourne à l'étape intermédiaire précédente, en marche arrière

        this->avoiding = false;
        statusStrat=1;
    }
    else
    {
        #ifdef QTGUI
        qDebug() << "Pas en train d'eviter";
        #endif

        //On reset toute les directions à aller en marche avant
        for(int i = 0 ; i < this->nombreEtapes ; i++)
        {
            /*actionGoto[i].setGoBack(false);
            actionEtape[i]->setGoBack(false);*/
            tableauEtapesTotal[i]->setGoBack(false);
        }
        this->enTrainEviterReculant = false;
        this->enTrainEviterAvancant = false;

        if(this->statusStrat==2)//Si on vient d'arriver à une étape intermédiare
        {
            this->updateIntermedaire();
        }
        else//Sinon, statusStrat==1, et il faut donc choisir un nouvel objectif
        {
            // Si on n'était pas en train d'éviter
            if(!(enTrainEviterReculant || enTrainEviterAvancant))
            {
                //L'objectif qu'on vient de remplir est maintenant un simple point de passage
                //this->tableauEtapesTotal[this->etapeEnCours]->finir();//Vieux, on utilise maintenant updateStock dans la strat de l'année en cours
                //Idem pour les autres étapes correspondant au même objectif
                for(int etapeLiee = 0 ; etapeLiee < this->tableauEtapesTotal[this->etapeEnCours]->getNombreEtapesLieesParFinirEtape() ; etapeLiee++)
                {
                    int numeroEtapeLiee = this->tableauEtapesTotal[this->etapeEnCours]->getEtapesLieesParFinirEtape()[etapeLiee];
                    this->tableauEtapesTotal[numeroEtapeLiee]->finir();
                }

                //Mise à jour du stock et l'objectif qu'on vient de remplir est maintenant un simple point de passage
                this->updateStock();

                //On est maintenant arrivé à l'étape de fin de l'action (en général c'est la même étape, mais pas toujours, ex : les claps de 2015)
                this->etapeEnCours = this->tableauEtapesTotal[this->etapeEnCours]->getNumeroEtapeFinAction() == -1
                                        ? this->etapeEnCours
                                        : this->tableauEtapesTotal[this->etapeEnCours]->getNumeroEtapeFinAction();

            }


            int score = 0;
            bool resteDesChosesAFaire = updateScores();


            // S'il n'y a plus d'objectif dans tout le graphe, on se replit vers une position où on ne bloque pas l'adversaire.
            // Sinon, il y a risque de prendre un avertissement pour anti-jeu (évité de peu pour le premier match de Krabi 2014)
            if(!resteDesChosesAFaire)
            {
                for(int i = 0 ; i < this->nombreEtapes ; i++)
                {
                    this->tableauEtapesTotal[i]->oublieRobotVu();
                }
                resteDesChosesAFaire = updateScores();

                //S'il n'y a VRAIMENT plus rien à faire
                if(!resteDesChosesAFaire)
                {
                    //Si on est au garage, on s'arrête
                    if(this->etapeEnCours == this->numeroEtapeGarage)
                    {
                        this->statusStrat=-1;//Plus rien à faire, on passe à l'action suivante de stratégieV2
#ifdef QTGUI
                        qDebug() << "Fin de StrategieV3";
#endif
                        return this->statusStrat;
                    }
                    else
                    {
                        //Sinon on y va
                        this->tableauEtapesTotal[this->numeroEtapeGarage]->setScore(1000);
                    }
                }

            }

            //On recalcul les distances par rapport à l'étape où l'on vient d'arriver
            dijkstra->setEtapeCourante(this->etapeEnCours);

            if(dijkstra->run() != 0)
            {
                // Si run renvoit autre chose que 0, c'est que l'étape en cours a changée.
                // Cela arrive pour débloquer le robot
                //Etape* ancienneEtape = this->tableauEtapesTotal[this->etapeEnCours];
                //this->etapeEnCours = this->tableauEtapesTotal[this->etapeEnCours]->getParent()->getNumero();
                tableauEtapesTotal[this->etapeEnCours]->getAction()->reset();
            }

            //On sélectionne l'objectif le plus prometteur : pas trop loin et qui rapporte
            int meilleurEtape = -1;
            int scoreMaxi = -100000;

            int scoreTypeEtape = 0;
            for(int i = 0 ; i < this->nombreEtapes ; i++)
            {
                scoreTypeEtape = this->tableauEtapesTotal[i]->getScore();
        //        score = modificateurTemporel*(10000-this->tableauEtapesTotal[i]->getDistance() + scoreTypeEtape);
                score = (10000-this->tableauEtapesTotal[i]->getDistance() + scoreTypeEtape);
                if((scoreMaxi < score) && scoreTypeEtape && (this->tableauEtapesTotal[i]->getDistance() != -1))
                {
                    scoreMaxi = score;
                    meilleurEtape = i;
                }
            }

            if(meilleurEtape==-1)
            {
                if(this->etapeEnCours == this->numeroEtapeGarage)
                {
                    this->statusStrat=-1;//Plus rien à faire, on passe à l'action suivante de stratégieV2
                    return this->statusStrat;
                }
                else
                {
                    meilleurEtape = this->numeroEtapeGarage;
                }
            }

            this->goal = meilleurEtape;
            this->statusStrat = 2;//Jusqu'à preuve du contraire, la prochaine étape est une étape intermédiaire
            this->updateIntermedaire();//On y va
        }
    }

    return this->statusStrat;
}

void StrategieV3::resetEverything(){
    for(int i = 0 ; i < 10 ; i++){
        this->tableauEtapesTotal[i]->setState(0);
    }
}

void StrategieV3::collisionAvoided(){
    this->avoiding = true;
}


void StrategieV3::updateIntermedaire()
{
    //Note : le parent d'une étape est l'étape le rapprochant le plus de l'étape d'origine
    //Ainsi, le parent du parent du parent... de n'importe quelle étape est l'étape d'origine
    //(sauf peut être le parent de l'étape d'origine, mais on s'en fout

    #ifdef QTGUI
        qDebug() << "updateIntermedaire\n";
    #endif
    int etapeOuOnVientDArriver = this->etapeEnCours;
    this->etapeEnCours = this->goal;
    this->nextStep = -1;

    // Si la prochaine étape est le goal, alors au prochain update il faudra trouver un nouvel objectif -> status = 1;
    if(((this->tableauEtapesTotal[this->etapeEnCours]->getParent()->getNumero())) == etapeOuOnVientDArriver)
    {
        #ifdef QTGUI
            qDebug() << "la prochaine etape est le goal\n" << etapeOuOnVientDArriver;
        #endif
        this->statusStrat = 1;
    }


    //On cherche l'etape suivant vers l'etape - but
    while(((this->tableauEtapesTotal[this->etapeEnCours]->getParent()->getNumero())) != etapeOuOnVientDArriver)
    {

        this->nextStep = this->etapeEnCours;
        this->etapeEnCours = ((this->tableauEtapesTotal[this->etapeEnCours]->getParent()->getNumero()));

    }

    if(this->statusStrat == 1)
    {
        //On réalise l'action de l'étape - but
        #ifndef STANDALONE_STRATEGIE
            StrategieV2::addTemporaryAction(tableauEtapesTotal[this->etapeEnCours]->getAction());
        #else
            // @TODO add temp action
        #endif // STANDALONE_STRATEGIE

    }
    else
    {
        //On ajoute l'action d'aller en ligne droite vers cette étape intermédiaire
#ifdef SMOOTH_MOVE
        tableauEtapesTotal[this->etapeEnCours]->getActionGoTo()->setNextGoal(tableauEtapesTotal[this->nextStep]->getPosition());
#endif

        #ifndef STANDALONE_STRATEGIE
            StrategieV2::addTemporaryAction(tableauEtapesTotal[this->etapeEnCours]->getActionGoTo());
        #else
            // @TODO add temp action
        #endif // STANDALONE_STRATEGIE

    }
}


#ifdef QTGUI
void StrategieV3::paint(QPainter* p)
{
    for(int numeroEtape = 0 ; numeroEtape<this->nombreEtapes ; numeroEtape++)
    {
        QPoint position = QPoint(
        this->tableauEtapesTotal[numeroEtape]->getPosition().getX(),
        this->tableauEtapesTotal[numeroEtape]->getPosition().getY());

        // Affichage des étapes
        p->setPen(colorEtapesIntermediaires);//"orange"));
        p->setBrush(colorEtapesIntermediaires);//"orange"));
        p->setOpacity(1.0f);
        p->drawEllipse(position,10,10);

        // Etape - but en surbrillance
        if(numeroEtape == this->goal)
        {
            p->setPen(colorEtapeGoal);
            p->setBrush(colorEtapeGoal);
            p->setOpacity(1.0f);
            p->drawEllipse(position,30,30);
        }

        // Etape actuelle en surbrillance
        if(numeroEtape == this->etapeEnCours)
        {
            p->setPen(colorEtapesIntermediaires);
            p->setBrush(colorEtapesIntermediaires);
            p->setOpacity(1.0f);
            p->drawEllipse(position,25,25);
        }

        // Etapes où on a vu un robot
        if(this->tableauEtapesTotal[numeroEtape]->aEviter())
        {
            p->setPen(colorEtapesRobotVu);
            p->setBrush(colorEtapesRobotVu);
            p->setOpacity(1.0f);
            p->drawEllipse(position,20,20);
        }

        //Affichage du numéro des étapes
        QFont font;
        font.setPixelSize(50);
        p->setFont(font);
        p->setOpacity(1);
        p->setPen(colorTexteEtapes);
        p->setBrush(colorTexteEtapes);
        p->drawText(position, QString::number(this->tableauEtapesTotal[numeroEtape]->getNumero()));

        // Affichage des liaisons entre étapes
        for(int numeroChild = 0 ; numeroChild < this->tableauEtapesTotal[numeroEtape]->getNbChildren() ; numeroChild++)
        {
//            Affichages de liaisons
//            QPoint positionChild = QPoint(
//            this->tableauEtapesTotal[numeroEtape]->getChildren()[numeroChild]->getPosition().getX(),
//            -(this->tableauEtapesTotal[numeroEtape]->getChildren()[numeroChild]->getPosition().getY()));

            // Affichage des demi-liaisons, pour mieux voir les liens mono-directionnels
            QPoint positionChild = QPoint(
            (this->tableauEtapesTotal[numeroEtape]->getChildren()[numeroChild]->getPosition().getX()+this->tableauEtapesTotal[numeroEtape]->getPosition().getX())/2,
            (this->tableauEtapesTotal[numeroEtape]->getChildren()[numeroChild]->getPosition().getY()+this->tableauEtapesTotal[numeroEtape]->getPosition().getY())/2);
            p->setOpacity(0.5f);
            p->setPen(colorLiaisonsEtapes);
            p->setBrush(colorLiaisonsEtapes);
            p->drawLine(position.x(), position.y(),positionChild.x(),positionChild.y());
            p->drawEllipse(position,10,10);
        }

        //Affichage du type d'étape
        QPoint positionTypeEtape = QPoint(
        this->tableauEtapesTotal[numeroEtape]->getPosition().getX(),
        this->tableauEtapesTotal[numeroEtape]->getPosition().getY() + 50);
        p->setFont(font);
        p->setOpacity(0.5);
        p->setPen(colorTexteEtapes);
        p->setBrush(colorTexteEtapes);
        p->drawText(positionTypeEtape, Etape::getShortNameType(this->tableauEtapesTotal[numeroEtape]->getEtapeType()));
    }
    p->setOpacity(1);
}
#endif

void StrategieV3::startDijkstra() {

    for(int i=0; i<this->nombreEtapes; i++)
        tableauEtapesTotal[i]->computeChildDistances();

    this->dijkstra = new Dijkstra(tableauEtapesTotal, this->nombreEtapes);

    tableauEtapesTotal[0]->setParent(tableauEtapesTotal[0]);// Evite de planter si on detecte dès la première boucle (dans le simu)

    dijkstra->setEtapeCourante(0);
}

void StrategieV3::updateStock(){
    switch(this->tableauEtapesTotal[this->etapeEnCours]->getEtapeType()){
        case Etape::DEPART :
            this->tableauEtapesTotal[this->etapeEnCours]->setEtapeType(Etape::POINT_PASSAGE);
            break;
        case Etape::PRE_BAFFE :
            this->tableauEtapesTotal[this->etapeEnCours]->setEtapeType(Etape::POINT_PASSAGE);
            break;
        default:
            this->tableauEtapesTotal[this->etapeEnCours]->setEtapeType(Etape::POINT_PASSAGE);
    }
}

bool StrategieV3::updateScores() {
    //Mise à jour du score correspondant à chaque étape
    //CETTE PARTIE EST A AMELIORER
    //on pourrait prendre en compte :
    //le fait le transporter un feu, ce qui active les objectifs de dépose done
    //le temps que prend chaque action
    //le temps restant done
    //la trajectoire de notre autre robot
    //...
    //C'est aussi utilisé pour savoir si on est dans un sous-graphe sans objectif,
    //et qu'il faut oublier qu'on a vu des robots
    int scoreTypeEtape = 0;
    //float modificateurTemporel = 1.f;

    bool resteDesChosesAFaire = false;
    for(int i = 0 ; i < this->nombreEtapes ; i++)
    {
        scoreTypeEtape=this->getScoreEtape(i);
        if(scoreTypeEtape)
        {
            resteDesChosesAFaire = true;
        }
        this->tableauEtapesTotal[i]->setScore(scoreTypeEtape);
    }
    return resteDesChosesAFaire;
}
