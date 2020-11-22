//Prononcer Da_ik_stra
//http://fr.wikipedia.org/wiki/Algorithme_de_Dijkstra

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "krabilib/strategie/etape.h"


class Dijkstra
{
public:
    /** @brief Constructeur de Dijkstra *
    * @param tableauEtapes le tableau contenant toutes les étapes du graphe à parcourir *
    * @param nbEtapes le nombre d'étapes contenues dans le tableau */
    Dijkstra(std::vector<Etape*>& tableauEtapes, int nbEtapes);

    /** @brief Renvoi un pointeur vers l'étape courante *
    * @return pointeur vers l'étape courante */
    Etape* getEtapeCourante();

    /** @brief Calcul et renvoi la distance à vol d'oiseau entre deux étapes *
    * @param etapeDepart l'étape de départ *
    * @param etapeArrivee l'étape d'arrivée *
    * @return distance  à vol d'oiseau entre deux étapes*/
    static int calculDistanceDirect(Etape* etapeDepart, Etape* etapeArrivee);

    /** @brief Set l'étape courante *
    * @param numeroEtapeCourante le numéro de l'étape courante */
    void setEtapeCourante(int numeroEtapeCourante);

    /** @brief Fonction principale, applique l'algorithme de Dijkstra à toutes les étapes du tableau, *
    * en mettant à jour leur distance par rapport à l'étape courante *
    * @return 0 si tout va bien, -1 si l'étape en cours a changée (ce qui arrive pour décoincer le robot) */
    int run();

    /** @brief Décomposition de la fonction run, pour ne faire qu'une seule étape à la fois,
    * et ainsi tenir dans un temps de traitement de 5 ms *
    * @param step le numéro du miniRun *
    * @return -1 en cas de graphe bloqué */
    int miniRun(int step);

    /** @brief Renvoi le numéro de l'étape la plus proche dont on n'a pas encore mis à jour les voisins *
    * @param classementEtapeMinimale sert à indiquer le classement de l'étape trouvée (de la plus proche à la plus éloignée de l'étape origine *
    * @return le numéro de l'étape la plus proche dont on n'a pas encore mis à jour les voisins */
    int trouverMin(int classementEtapeMinimale);

    /** @brief Met à jour les voisin de l'étape spécifiée. Cela correspond à mettre à jour leur distance à l'origine si elle n'existait pas, ou bien si passer par cette étape est plus court que le chemin envisagé jusqu'alors *
    * @param numeroEtape le numéro de l'étape dont il faut mettre à jour les voisins */
    void mettreAJourVoisins(int numeroEtape);

    /** @brief Renvoi la distance de l'étape spécifiée à l'étape origine *
    * @param numeroEtape le numéro de l'étape dont on veut connaître la distance *
    * @return la distance de l'étape spécifiée à l'étape origine */
    int getDistance(int numeroEtape);

    /** @brief Renvoi la distance de l'étape spécifiée à l'étape origine *
    * @param etape l'étape dont on veut connaître la distance *
    * @return la distance de l'étape spécifiée à l'étape origine */
    int getDistance(Etape* etape);

    /** @brief Renvoi le nombre d'etapes constituant de graphe *
    * @return le nombre d'etapes constituant de graphe */
    int getNombreEtapes();

    /** @brief Reset l'index de progression pour miniRun */
    void resetProgress();

    /** @brief Renvoi un booléen valait true si cette étape peut rapporter des points */
    bool etapeRapporte(Etape* etape);

private:
    /** @brief (Re-)initialise le tableau pour fiare une nouvelle recherche */
    void initialiser();

    /** @brief le tableau des etapes constituant le graph à explorer */
    std::vector<Etape*>& tableauEtapes;

    /** @brief le nombre d'étapes constituant le graph */
    int nombreEtapes;

    /** @brief le numero de l'étape courante */
    int numeroEtapeCourante;

    /** @brief index de progression pour miniRun */
    int updateInProgress;

    /** @brief nombre de fruits transporte par le robot */
    int nombreFruitsTransporte;

    /** @brief nombre de feux transporte par le robot */
    int nombreFeuxTransporte;

    /** @brief nombre de fresques transporte par le robot */
    int nombreFresquesTransporte;

    /** @brief nombre de lances transporte par le robot */
    int nombreLancesTransporte;
    int nombreFiletTransporte;
};

#endif // DIJKSTRA_H
