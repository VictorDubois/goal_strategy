#pragma once

#include "krabilib/position.h"
#include <vector>

#define ETAPE_INVALID_IDX -1

class MediumLevelAction;

class Etape
{
public:
    /** @brief Enum des types d'étape possible (un feu, un point de passage...) */
    enum EtapeType
    {
        DEPART = 0x01,
        // 2025
        STOCK_MATIERE_PREMIERE = 0x26,
        AIRE_DE_CONSTRUCTION = 0x27,
        GARDE_MANGER = 0x28,
        ZONE_DE_RAMASSAGE = 0x29,
        THERMOMETRE = 0x2A,

        POINT_PASSAGE = 0x30,

        ROBOT_VU_ICI = 0x60

    };

    static int makeEtape(MediumLevelAction* action);
    static int makeEtape(MediumLevelAction* action, std::string a_name);
    static int makeEtape(Position position, std::string a_name, EtapeType type = POINT_PASSAGE);

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
    float getDistance();

    /** @brief Set la distance de cette étape à l'étape où se trouve le robot *
     *   @param distance de cette étape à l'étape où se trouve le robot */
    void setDistance(float distance);

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
    float* getDistances();

    /** @brief set un tableau des distances vers les etapes attachees a celle-ci *
     *   @param distances Tableau des distances vers les etapes attachées à celle-ci */
    void setDistances(float* distances);

    void computeChildDistances();

    /** @brief Renvoi un tableau des étapes qui doivent être considérées comme finie si celle-ci
     * l'est */
    std::vector<int> getEtapesLieesParFinirEtape();

    /** @brief ajoute dans un vector une étape qui doit être considérée comme finie si celle-ci
     * l'est
     * *
     *   @param children étape qui doit être considérée comme finie si celle-ci
     * l'est */
    void addEtapeLieeParFinirEtape(int numerosEtapeLieesParFinirEtape);

    /** @brief set un vector des étapes qui doivent être considérées comme finie si celle-ci l'est
     * *
     *   @param children vector des étapes qui doivent être considérées comme finie si celle-ci
     * l'est */
    void setEtapesLieesParFinirEtape(std::vector<int> numerosEtapesLieesParFinirEtape);

    /** @brief Renvoi le nombre d'étapes qui doivent être considérées comme finie si celle-ci l'est
     */
    int getNombreEtapesLieesParFinirEtape();

    /** @brief Effectue les changements nécessaires pour considérer l'étape comme effectuée */
    void finir(void);

    /** @brief set le score de l'étape d'après l'heuristique prenant en compte la distance *
     *   @param score le score heuristique de l'étape */
    void setHeuristicScore(int score);

    /** @brief Renvoi le score de l'étape d'après l'heuristique prenant en compte la distance */
    int getHeuristicScore();

    /** @brief set le score de l'étape *
     *   @param score le score de l'étape */
    void setScore(int score);

    /** @brief Renvoi le score de l'étape */
    int getScore();

    /** @brief Renvoi le nom de l'étape */
    std::string getName();

    std::string getCustomName()
    {
        return m_custom_name;
    };

    void setCustomName(std::string a_custom_name)
    {
        m_custom_name = a_custom_name;
    };

    /** @brief Ajoute un voisin au tableau de voisins */
    void addVoisin(Etape* newVoisin, bool autreSens = true);

    /** @brief Ajoute un voisin au tableau de voisins */
    void addVoisin(int newVoisinIndex, bool autreSens = true);

    void addVoisins(int newVoisinIndex);
    void addVoisins(int newVoisinIndex, int newVoisinIndex2);
    void addVoisins(int newVoisinIndex, int newVoisinIndex2, int newVoisinIndex3);
    void addVoisins(int newVoisinIndex,
                    int newVoisinIndex2,
                    int newVoisinIndex3,
                    int newVoisinIndex4);
    void addVoisins(int newVoisinIndex,
                    int newVoisinIndex2,
                    int newVoisinIndex3,
                    int newVoisinIndex4,
                    int newVoisinIndex5);
    void addVoisins(int newVoisinIndex,
                    int newVoisinIndex2,
                    int newVoisinIndex3,
                    int newVoisinIndex4,
                    int newVoisinIndex5,
                    int newVoisinIndex6);

    /** @brief Ajoute un voisin au tableau de voisins */
    /*void addVoisins();
    template<typename... Types>
    void addVoisins(int newVoisinIndex1, Types... otherIndexes)
    {
        this->addVoisin(newVoisinIndex1);
        addVoisins(otherIndexes...);
    }*/

    /** @brief Setter de l'étape à laquelle on fini l'action de l'étape */
    void setNumeroEtapeFinAction(int newNumeroEtapeFinAction);

    /** @brief Getter de l'étape à laquelle on fini l'action de l'étape */
    int getNumeroEtapeFinAction();

    void setAction(MediumLevelAction* action);

    MediumLevelAction* getAction();

    void reset();

    void setGoBack(bool val);

    void setDistanceToPotentialObstacle(float distance)
    {
        distanceToPotentialObstacle = distance;
    };

    float getDistanceToPotentialObstacle()
    {
        return distanceToPotentialObstacle;
    };

    static int getTotalEtapes();

    static std::vector<Etape*>& initTableauEtapeTotal(int number);

    static Etape* get(int index);

    static std::vector<Etape*>& getTableauEtapesTotal();

    void setEtapesActiveesApres(std::vector<Etape*> a_etapes_a_activer_apres);

    void active();

    void desactive();

    void activeEtapesApres();

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
    float distance;

    /** @brief Type d'étape (un feu, un point de passage...) */
    EtapeType type;

    /** @brief int Set le nombre d'étapes attachées à celle_ci */
    int nbChildren;

    /** @brief int le numero de cette étape */
    int numero;

    /** @brief Tableau des distances vers les étapes attachées à celle-ci */
    float* distances;

    /** @brief Nombre d'étapes qui doivent être considérées comme finie si celle-ci l'est */
    int nombreEtapesLieesParFinirEtape;

    /** @brief Tableau des étapes qui doivent être considérées comme finie si celle-ci l'est */
    std::vector<int> numerosEtapesLieesParFinirEtape;

    /** @brief Score de l'étape, correspondant à si on veut que le robot la réalise ou pas*/
    int score;

    /** @brief Score de l'étape, correspondant à si on veut que le robot la réalise ou pas en
     * prenant en compte la distance*/
    int heuristicScore;
    /** @brief Etape à laquelle on fini l'action de l'étape */
    int numeroEtapeFinAction;

    /** @brief Distance à l'obstacle (potentiel autre robot) le plus proche */
    float distanceToPotentialObstacle;

    MediumLevelAction* action;

    // static int numberInit;

    static int totalEtapesInstanciated;

    static std::vector<Etape*> tableauEtapesTotal;

    void postInit();

    std::string m_custom_name = "Undefined";

    bool m_active = true;
    std::vector<Etape*> m_etapes_a_activer_apres;
};
