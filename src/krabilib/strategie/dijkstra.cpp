//Prononcer Da_ik_stra
//http://fr.wikipedia.org/wiki/Algorithme_de_Dijkstra

#include "krabilib/strategie/dijkstra.h"
#include <math.h>

Dijkstra::Dijkstra(){}


Dijkstra::Dijkstra(Etape** tableauEtapes, int nbEtapes)
{
    this->tableauEtapes = tableauEtapes;
    this->nombreEtapes = nbEtapes;
}

Etape* Dijkstra::getEtapeCourante()
{
    return tableauEtapes[this->numeroEtapeCourante];
}

int Dijkstra::calculDistanceDirect(Etape* etapeDepart, Etape* etapeArrivee)
{
    //distance quartésienne : d = sqrt(x^2+y^2)
    //sqrt = square root, racine carrée
    //Cette étape est très gourmande en ressources, même si les calculs sont fait sur des entiers.
    //Une solution, vu que le STM32-H405 a plein de mémoire, serait de pré-calculer les distances, et de les stocker.
    //Cela permettrait aussi d'ajuster certaines distances si besoin, pour préférer certains passages à d'autres
    return sqrt(
                    pow(fabs((int) etapeDepart->getPosition().getX()-etapeArrivee->getPosition().getX()),2)+
                    pow(fabs((int) etapeDepart->getPosition().getY()-etapeArrivee->getPosition().getY()),2)
                    );
}

void Dijkstra::setEtapeCourante(int numeroEtapeCourante)
{
    this->numeroEtapeCourante = numeroEtapeCourante;
}

//Calcul des distances de toutes les étapes par rapport à l'étape en cours (étape origine)
int Dijkstra::run()
{
  /*  int step = 0;
    int resultatMiniRun = -2;
    while(resultatMiniRun == -2)
    {
        resultatMiniRun = miniRun(step);
        step++;
    }

    if(resultatMiniRun == -1)
        return -1;
    else
        return 0;*/



    int min = -2;
    initialiser();

    mettreAJourVoisins(this->numeroEtapeCourante);

    for(int i = 0 ; i < this->nombreEtapes-2 ; i++)
    {
        min = trouverMin(i+1);
        // Si min vaut -1, c'est que le robot était coincé, et donc qu'il a fallut réinitialiser la recherche
        // Donc on arrete la recherche en cours, une nouvelle a été lancée
        if(min == -1)
        {
            return -1;
        }
        if(min == -2)
        {
            return 0;
        }
        this->numeroEtapeCourante = min;
        mettreAJourVoisins(this->numeroEtapeCourante);
    }
    return 0;
}

int Dijkstra::miniRun(int step)
{
    if (step == 0)
    {
        initialiser();
    }
    else if(step < this->nombreEtapes-2)
    {
        int min = trouverMin(step+1);
        // Si min vaut -1, c'est que le robot était coincé, et donc qu'il a fallut réinitialiser la recherche
        // Donc on arrete la recherche en cours, une nouvelle a été lancée
        if(min == -1)
        {
            return -1;
        }
        if(min == -2)
        {
            return 0;
        }
        this->numeroEtapeCourante = min;
        mettreAJourVoisins(this->numeroEtapeCourante);
    }
    else
    {
        return 0;
    }
    return -2;
/*
    int min = -2;


    mettreAJourVoisins(this->numeroEtapeCourante);

    for(int i = 0 ; i < this->nombreEtapes-2 ; i++)
    {
        min = trouverMin(i+1);
        // Si min vaut -1, c'est que le robot était coincé, et donc qu'il a fallut réinitialiser la recherche
        // Donc on arrete la recherche en cours, une nouvelle a été lancée
        if(min == -1)
        {
            return -1;
        }
        if(min == -2)
        {
            return 0;
        }
        this->numeroEtapeCourante = min;
        mettreAJourVoisins(this->numeroEtapeCourante);
    }
    return 0;*/
}

//On réinitialise le tableau avant le commencer une nouvelle recherche
void Dijkstra::initialiser()
{
    for(int i = 0 ; i<this->nombreEtapes ; i++)
    {
        this->tableauEtapes[i]->setDistance(-1);
        if(!(this->tableauEtapes[i]->aEviter()))//getState() != -2)
        {
            this->tableauEtapes[i]->setState(-1);
        }
        else
        {
            this->tableauEtapes[i]->setState(-2);
        }
    }
    this->tableauEtapes[this->numeroEtapeCourante]->setDistance(0);
    this->tableauEtapes[this->numeroEtapeCourante]->setState(0);
}

// Renvoi le numéro de l'étape la plus proche dont on n'a pas encore mis à jour les voisins
int Dijkstra::trouverMin(int classementEtapeMinimale)
{
    int minimum = -1;
    int numeroEtapeLaPlusProche = -1;

    for(int i = 0 ; i < this->nombreEtapes ; i++)
    {
        if(this->tableauEtapes[i]->getState()==-1 && (!(this->tableauEtapes[i]->aEviter())) && (!(this->tableauEtapes[i]->getDistance()==-1)) && (minimum == -1 || this->tableauEtapes[i]->getDistance() < minimum))
        {
            minimum = this->tableauEtapes[i]->getDistance();
            numeroEtapeLaPlusProche = i;
        }
    }
    /*TODO: WHAT THE HELL IS THAT ???*/
    //Du debug


    /*Etape* etape1 = this->tableauEtapes[0];
    Etape* etape2 = this->tableauEtapes[1];
    Etape* etape3 = this->tableauEtapes[2];
    Etape* etape4 = this->tableauEtapes[3];
    Etape* etape5 = this->tableauEtapes[4];
    Etape* etape6 = this->tableauEtapes[5];
    Etape* etape7 = this->tableauEtapes[6];
    Etape* etape8 = this->tableauEtapes[7];
    Etape* etape9 = this->tableauEtapes[8];
    Etape* etape10 = this->tableauEtapes[9];
    Etape* etape11 = this->tableauEtapes[10];
    Etape* etape12 = this->tableauEtapes[11];
    Etape* etape13 = this->tableauEtapes[12];
    Etape* etape14 = this->tableauEtapes[13];
    Etape* etape15 = this->tableauEtapes[14];
    Etape* etape16 = this->tableauEtapes[15];
    Etape* etape17 = this->tableauEtapes[16];
    Etape* etape18 = this->tableauEtapes[17];
    Etape* etape19 = this->tableauEtapes[18];
    Etape* etape20 = this->tableauEtapes[19];
    Etape* etape21 = this->tableauEtapes[20];
    Etape* etape22= this->tableauEtapes[21];
    Etape* etape23= this->tableauEtapes[22];
    Etape* etape24= this->tableauEtapes[23];
    Etape* etape25= this->tableauEtapes[24];
    Etape* etape26= this->tableauEtapes[25];
    Etape* etape27= this->tableauEtapes[26];
    Etape* etape28= this->tableauEtapes[27];
    Etape* etape29= this->tableauEtapes[28];
    Etape* etape30= this->tableauEtapes[29];
    Etape* etape31= this->tableauEtapes[30];
    Etape* etape32= this->tableauEtapes[31];
    Etape* etape33= this->tableauEtapes[32];
    Etape* etape34= this->tableauEtapes[33];
    Etape* etape35= this->tableauEtapes[34];
    Etape* etape36= this->tableauEtapes[35];
    Etape* etape37= this->tableauEtapes[36];
    Etape* etape38= this->tableauEtapes[37];
    Etape* etape39= this->tableauEtapes[38];*/
    //Si on ne trouve pas d'étape la plus proche, alors c'est qu'on est coincé par les autres robots.
    //Il faut donc faire des allers-retours entre les différentes étapes possibles.
    //Pour cela, on change le status des étapes "robot vu" à "point de passage", puis relancer une passe de Dijkstra (récursion powa!!!).
    if(numeroEtapeLaPlusProche == -1)
    {
        //On check si on a encore des points à marquer dans la zone ou on est confiné
        bool onEstCoinceDansUnEndroitPourri = true;
        for(int i = 0 ; i < this->nombreEtapes ; i++)
        {
            if(this->etapeRapporte(this->tableauEtapes[i]) && (!(this->tableauEtapes[i]->aEviter())) && this->tableauEtapes[i]->getDistance() != -1 && this->tableauEtapes[i]->getState() != -2)
            {
                onEstCoinceDansUnEndroitPourri = false;
            }
        }
        //Sinon on supprime les barrières en oubliant qu'on a vu des robots
        if(onEstCoinceDansUnEndroitPourri)
        {
            for(int i = 0 ; i < this->nombreEtapes ; i++)
            {
                //On reset numeroEtapeEnCours à l'étape où on est actuellement
                if(this->tableauEtapes[i]->getDistance() == 0)
                {
                    this->numeroEtapeCourante = i; //-1?
                }

                this->tableauEtapes[i]->oublieRobotVu();
            }

            this->run();
            return -1;
        }
        else
        {
            return -2;
        }
    }
    this->tableauEtapes[numeroEtapeLaPlusProche]->setState(classementEtapeMinimale);
    return numeroEtapeLaPlusProche;
}

//Met à jour les voisin de l'étape spécifiée.
void Dijkstra::mettreAJourVoisins(int numeroEtape)
{
    int distanceEtapeCourante = this->tableauEtapes[numeroEtape]->getDistance();
    int distanceChildCourant;
    int distanceEtapeVersChild;

    for(int i = 0 ; i < this->tableauEtapes[numeroEtape]->getNbChildren() ; i++)
    {
        //Si cette étape n'est pas bannie
        //if(this->tableauEtapes[numeroEtape]->getChild(i)->getState() > -2 && ((int) this->tableauEtapes[numeroEtape]->getChild(i)->getEtapeType() < 20))
        if(!(this->tableauEtapes[numeroEtape]->getChild(i)->aEviter()))
        {
            distanceChildCourant = this->tableauEtapes[numeroEtape]->getChild(i)->getDistance();
            distanceEtapeVersChild = this->tableauEtapes[numeroEtape]->getDistances()[i];
            //distanceEtapeVersChild = calculDistanceDirect(this->tableauEtapes[numeroEtape]->getChild(i), this->tableauEtapes[numeroEtape]);
            if(!distanceEtapeVersChild)
                distanceEtapeVersChild = 1;//Pas de distances nulles
            //Si le chemin vers ce voisin est plus court en passant par l'étape actuelle, mise à jour de ce voisin
            //Si le voisin n'avait pas encore été atteind par un chemin, alors on le met à jour aussi (distanceChildCourant == -1)
            if(distanceChildCourant == -1 || distanceChildCourant > distanceEtapeCourante + distanceEtapeVersChild)
            {
                this->tableauEtapes[numeroEtape]->getChild(i)->setDistance(distanceEtapeCourante + distanceEtapeVersChild);
                this->tableauEtapes[numeroEtape]->getChild(i)->setParent(this->tableauEtapes[this->numeroEtapeCourante]);
            }
        }
    }
}

int Dijkstra::getDistance(int numeroEtape)
{
    return tableauEtapes[numeroEtape]->getDistance();
}

int Dijkstra::getDistance(Etape* etape)
{
    return etape->getDistance();
}

bool Dijkstra::etapeRapporte(Etape* etape)
{
    // A segfault here can be related to a splitted graph (ex: start not linked to the rest of the graph)
    return (bool) etape->getScore();
}
