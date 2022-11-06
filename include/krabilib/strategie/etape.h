#pragma once

#include "krabilib/position.h"
#include <vector>

#define ETAPE_INVALID_IDX -1

class ActionGoTo;
class MediumLevelAction;

class Etape
{
public:
    /** @brief Enum des types d'étape possible (un feu, un point de passage...) */
    enum EtapeType
    {
        DEPART = 0x01,
        //#ifdef GOLDO2018

        ABEILLE = 0x03,
        RESERVOIR_EAU = 0x05,

        GOLDENIUM = 0x06,
        ACCELERATOR = 0x07,
        PRE_BAFFE = 0x09,
        //#elif KRABI2016
        CUBE_DEBUT = 0x02,

        DUNE = 0x04,
        ZONE_CONSTRUCTION = 0x08,
        CABINE = 0x10,

        PHARE = 0x11,
        MANCHE_A_AIR = 0x12,
        PORT = 0x13,
        MOUILLAGE_NORD = 0x14,
        MOUILLAGE_SUD = 0x15,
        BOUEE = 0x16,
        HEXAGON_PLAT = 0x17,
        CARRE_FOUILLE = 0x18,
        STATUETTE = 0x19,
        VITRINE = 0x20,
        GALERIE = 0x21,
        PILE_GATEAU = 0x22,
        ASSIETTE = 0x23,

        POINT_PASSAGE = 0x30,

        ROBOT_VU_ICI = 0x60

    };

    static int makeEtape(MediumLevelAction* action);
    static int makeEtape(Position position, EtapeType type = POINT_PASSAGE);

    /** @brief Renvoi un pointeur vers une des etapes attachees a celle-ci *
     *   @param nb le numéro du lien vers l'autre etape */
    Etape* getChild(int nb);

    /** @brief Renvoi un tableau de pointeurs vers les etapes attachees a celle-ci */
    Etape** getChildren();

    /** @brief Renvoi l'étape précédente pour remonter à l'étape en cours (utilisé pour
     * l'exploration du graphe) */
    Etape* getParent();

    /** @brief Renvoi la position de cette étape */
    Position getPosition();

    /** @brief Renvoi l'état de l'étape courante (utilisé pour l'exploration du graphe) */
    int getState();

    /** @brief Set l'état de l'étape courante (utilisé pour l'exploration du graphe) *
     *   @param state le nouvel état de l'étape */
    void setState(int state);

    /** @brief Renvoi la distance de cette étape à l'étape où se trouve le robot */
    int getDistance();

    /** @brief Set la distance de cette étape à l'étape où se trouve le robot *
     *   @param distance de cette étape à l'étape où se trouve le robot */
    void setDistance(int distance);

    /** @brief Set l'étape précédente pour remonter à l'étape en cours (utilisé pour l'exploration
     * du graphe) */
    void setParent(Etape* parent);

    /** @brief Renvoi le nombre d'étapes attachées à celle_ci */
    int getNbChildren();

    /** @brief Renvoi le type de cette étape (un feu, un point de passage...) */
    EtapeType getEtapeType();

    /** @brief Set le type de cette étape (un feu, un point de passage...) *
     *   @param type le type de cette étape (un feu, un point de passage...) */
    void setEtapeType(EtapeType type);

    /** @brief set un tableau de pointeurs vers les etapes attachees a celle-ci *
     *   @param children Tableau des étapes attachées à celle-ci */
    void setChildren(Etape** children);

    /** @brief On aurait vu un robot sur le chemin de cette étape */
    void robotVu();

    /** @brief Renvoi le numero de cette étape */
    int getNumero();

    /** @brief Renvoi true s'il faut éviter cette étape */
    bool aEviter();

    /** @brief Oublie qu'on a vu un robot ici*/
    void oublieRobotVu();

    /** @brief Renvoi un tableau des distances vers les etapes attachees a celle-ci */
    int* getDistances();

    /** @brief set un tableau des distances vers les etapes attachees a celle-ci *
     *   @param distances Tableau des distances vers les etapes attachées à celle-ci */
    void setDistances(int* distances);

    void computeChildDistances();

    /** @brief Renvoi un tableau des étapes qui doivent être considérées comme finie si celle-ci
     * l'est */
    int* getEtapesLieesParFinirEtape();

    /** @brief set un tableau des étapes qui doivent être considérées comme finie si celle-ci l'est
     * *
     *   @param children Tableau des étapes qui doivent être considérées comme finie si celle-ci
     * l'est */
    void setEtapesLieesParFinirEtape(int* numerosEtapesLieesParFinirEtape);

    /** @brief Renvoi le nombre d'étapes qui doivent être considérées comme finie si celle-ci l'est
     */
    int getNombreEtapesLieesParFinirEtape();

    /** @brief Effectue les changements nécessaires pour considérer l'étape comme effectuée */
    void finir(void);

    /** @brief set le score de l'étape *
     *   @param score le score de l'étape */
    void setScore(int score);

    /** @brief Renvoi le score de l'étape */
    int getScore();

    /** @brief Ajoute un voisin au tableau de voisins */
    void addVoisin(Etape* newVoisin, bool autreSens = true);

    /** @brief Ajoute un voisin au tableau de voisins */
    void addVoisin(int newVoisinIndex, bool autreSens = true);

    /** @brief Ajoute un voisin au tableau de voisins */
    void addVoisins();
    template<typename... Types>
    void addVoisins(int newVoisinIndex1, Types... otherIndexes)
    {
        this->addVoisin(newVoisinIndex1);
        addVoisins(otherIndexes...);
    }

    /** @brief Setter de l'étape à laquelle on fini l'action de l'étape */
    void setNumeroEtapeFinAction(int newNumeroEtapeFinAction);

    /** @brief Getter de l'étape à laquelle on fini l'action de l'étape */
    int getNumeroEtapeFinAction();

    void setAction(MediumLevelAction* action);

    MediumLevelAction* getAction();

    ActionGoTo* getActionGoTo();

    void reset();

    void setGoBack(bool val);

    static int getTotalEtapes();

    static std::vector<Etape*>& initTableauEtapeTotal(int number);

    static Etape* get(int index);

    static std::vector<Etape*>& getTableauEtapesTotal();

private:
    /** @brief Constructeur d'une etape *
     *   @param position Position de cette étape *
     *   @param typeType d'étape (un feu, un point de passage...) *
     *   @param state Etat de cette étape, utilisé pour l'exploration du graphe
     *   @param nombreEtapesLieesParFinirEtape Nombre d'étapes qui doivent être considérées comme
     * finie si celle-ci l'est */
    Etape(Position position, EtapeType type = POINT_PASSAGE);

    /** @brief Tableau des étapes attachées à celle-ci */
    Etape** children;

    /** @brief Etape précédente pour remonter à l'étape en cours, utilisé pour l'exploration du
     * graphe */
    Etape* parent;

    /** @brief Position de cette étape */
    Position position;

    /** @brief Etat de cette étape, utilisé pour l'exploration du graphe */
    int state;

    /** @brief Distance de cette étape à l'étape où se trouve le robot */
    int distance;

    /** @brief Type d'étape (un feu, un point de passage...) */
    EtapeType type;

    /** @brief int Set le nombre d'étapes attachées à celle_ci */
    int nbChildren;

    /** @brief int le numero de cette étape */
    int numero;

    /** @brief Tableau des distances vers les étapes attachées à celle-ci */
    int* distances;

    /** @brief Nombre d'étapes qui doivent être considérées comme finie si celle-ci l'est */
    int nombreEtapesLieesParFinirEtape;

    /** @brief Tableau des étapes qui doivent être considérées comme finie si celle-ci l'est */
    int* numerosEtapesLieesParFinirEtape;

    /** @brief Score de l'étape, correspondant à si on veut que le robot la réalise ou pas*/
    int score;

    /** @brief Etape à laquelle on fini l'action de l'étape */
    int numeroEtapeFinAction;

    ActionGoTo* actionGoTo;

    MediumLevelAction* action;

    // static int numberInit;

    static int totalEtapesInstanciated;

    static std::vector<Etape*> tableauEtapesTotal;

    void postInit();
};
