#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/dijkstra.h"
#include "krabilib/strategie/mediumLevelAction.h"

std::vector<Etape*> Etape::tableauEtapesTotal;
int Etape::totalEtapesInstanciated = 0;

Etape::Etape(Position position, EtapeType type)
{
    int idx = (Etape::totalEtapesInstanciated++);

    this->position = position;
    this->type = type;
    this->state = -1;
    this->action = 0;
    this->nbChildren = 0;
    this->distance = -1;
    this->score = 0;
    this->numero = idx;
    this->numeroEtapeFinAction = idx;
    this->nombreEtapesLieesParFinirEtape = 0;
    this->distanceToPotentialObstacle = 5000;
    this->numerosEtapesLieesParFinirEtape.clear();
    this->numerosEtapeActiveApres.clear();

    if (idx != ETAPE_INVALID_IDX)
        tableauEtapesTotal[idx] = this;
}

std::string Etape::getName()
{
    switch (getEtapeType())
    {
    case EtapeType::DEPART:
        return "Depart";
        break;
    case EtapeType::POINT_PASSAGE:
        return "PP";
        break;
    case EtapeType::ROBOT_VU_ICI:
        return "OBS";
        break;
    case EtapeType::ZONE_DE_RAMASSAGE:
        return "ZdR";
        break;
    case EtapeType::GARDE_MANGER:
        return "GM";
        break;
    case EtapeType::THERMOMETRE:
        return "Thermo";
        break;
    default:
        return "";
        break;
    }
}

int Etape::makeEtape(MediumLevelAction* action, std::string a_name)
{
    auto new_etape = makeEtape(action);
    Etape::get(new_etape)->setCustomName(a_name);
    return new_etape;
}

int Etape::makeEtape(Position position, std::string a_name, EtapeType type)
{
    auto new_etape = makeEtape(position, type);
    Etape::get(new_etape)->setCustomName(a_name);
    return new_etape;
}

int Etape::makeEtape(MediumLevelAction* action)
{
    int idx = Etape::makeEtape(Position(), Etape::POINT_PASSAGE);

    if (idx == ETAPE_INVALID_IDX)
        return idx;

    Etape::get(idx)->setAction(action);

    return idx;
}

int Etape::makeEtape(Position position, EtapeType type)
{
    Etape* e = new Etape(position, type);

    int idx = e->getNumero();

    if (idx == ETAPE_INVALID_IDX)
        delete e;

    return idx;
}

int Etape::getTotalEtapes()
{
    return Etape::totalEtapesInstanciated;
}

Etape* Etape::getChild(int nb)
{
    return this->children[nb];
}

Etape** Etape::getChildren()
{
    return this->children;
}

Etape* Etape::getParent()
{
    return this->parent;
}

Position Etape::getPosition()
{
    return this->position;
}

int Etape::getState()
{
    return this->state;
}

void Etape::setState(int state)
{
    this->state = state;
}

float Etape::getDistance()
{
    return this->distance;
}

void Etape::setDistance(float distance)
{
    this->distance = distance;
}

void Etape::setParent(Etape* parent)
{
    this->parent = parent;
}

int Etape::getNbChildren()
{
    return this->nbChildren;
}

void Etape::setChildren(Etape** children)
{
    this->children = children;
}

Etape::EtapeType Etape::getEtapeType()
{
    return this->type;
}

void Etape::setEtapeType(Etape::EtapeType type)
{
    this->type = type;
}

void Etape::robotVu()
{
    if (!aEviter())
    {
        this->type = (EtapeType)(this->type + ROBOT_VU_ICI);
    }
}

int Etape::getNumero()
{
    return this->numero;
}

bool Etape::aEviter()
{
    if (((int)this->getEtapeType()) > ROBOT_VU_ICI - 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Etape::oublieRobotVu()
{
    if (this->aEviter() && this->getEtapeType() < Etape::EtapeType::PIVOT_DESACTIVEE)
    {
        // On oublie qu'on a vu un robot
        this->setEtapeType((EtapeType)((int)this->getEtapeType() - ROBOT_VU_ICI));
    }
}

float* Etape::getDistances()
{
    return this->distances;
}

void Etape::setDistances(float* distances)
{
    this->distances = distances;
}

void Etape::computeChildDistances()
{
    if (nbChildren == 0)
        return;

    this->distances = new float[this->nbChildren];

    for (int i = 0; i < this->nbChildren; ++i)
    {
        this->distances[i] = Dijkstra::calculDistanceDirect(this->children[i], this);

        if (this->children[i]->getDistanceToPotentialObstacle() < 0.1f)
        {
            this->distances[i] += 100; // mm?
        }
    }
}

std::vector<int> Etape::getEtapesLieesParFinirEtape()
{
    return this->numerosEtapesLieesParFinirEtape;
}

void Etape::addEtapeLieeParFinirEtape(int numerosEtapeLieesParFinirEtape)
{
    this->numerosEtapesLieesParFinirEtape.push_back(numerosEtapeLieesParFinirEtape);
}

std::vector<int> Etape::getEtapeActiveApres()
{
    return this->numerosEtapeActiveApres;
}

void Etape::addEtapeActiveApres(int a_numeroEtapeActiveApres)
{
    this->numerosEtapeActiveApres.push_back(a_numeroEtapeActiveApres);
    if (Etape::get(a_numeroEtapeActiveApres)->getEtapeType() < Etape::EtapeType::PIVOT_DESACTIVEE)
    {
        Etape::get(a_numeroEtapeActiveApres)
          ->setEtapeType((EtapeType)(Etape::get(a_numeroEtapeActiveApres)->getEtapeType()
                                     + Etape::EtapeType::PIVOT_DESACTIVEE));
    }
}

void Etape::setEtapesLieesParFinirEtape(std::vector<int> numerosEtapesLieesParFinirEtape)
{
    this->numerosEtapesLieesParFinirEtape = numerosEtapesLieesParFinirEtape;
}

int Etape::getNombreEtapesLieesParFinirEtape()
{
    return this->numerosEtapesLieesParFinirEtape.size();
}

void Etape::finir(void)
{
    this->setEtapeType(Etape::POINT_PASSAGE);
    return;
}

void Etape::setScore(int score)
{
    this->score = score;
}

int Etape::getScore()
{
    if (!m_active)
    {
        return 0;
    }
    return this->score;
}

void Etape::setHeuristicScore(int score)
{
    this->heuristicScore = score;
}

int Etape::getHeuristicScore()
{
    if (!m_active)
    {
        return 0;
    }
    return this->heuristicScore;
}

void Etape::setEtapesActiveesApres(std::vector<Etape*> a_etapes_a_activer_apres)
{
    m_etapes_a_activer_apres = a_etapes_a_activer_apres;
}

void Etape::active()
{
    m_active = true;
}

void Etape::desactive()
{
    m_active = false;
}

void Etape::activeEtapesApres()
{
    for (auto* l_etape : m_etapes_a_activer_apres)
    {
        l_etape->active();
    }
}

void Etape::setAction(MediumLevelAction* action)
{
    this->action = action;

    if (this->position == Position())
    {
        this->position = action->getGoalPosition();
        this->type = action->getType();
    }
}

MediumLevelAction* Etape::getAction()
{
    return this->action;
}

void Etape::addVoisin(Etape* newVoisin, bool autreSens)
{
    if (this->nbChildren == 0)
    {
        this->children = new Etape*[1];
        this->children[0] = newVoisin;
        this->nbChildren++;
    }
    else
    {
        Etape** temp = new Etape*[nbChildren];
        for (int i = 0; i < nbChildren; i++)
        {
            temp[i] = this->children[i];
        }
        this->children = new Etape*[nbChildren + 1];
        for (int i = 0; i < nbChildren; i++)
        {
            this->children[i] = temp[i];
        }
        delete[] temp;
        this->children[nbChildren] = newVoisin;
        this->nbChildren++;
    }

    if (autreSens)
    {
        newVoisin->addVoisin(this, false);
    }
}

void Etape::reset()
{
    if (this->action != 0)
        this->action->reset();
}

void Etape::setGoBack(bool val)
{
    if (this->action != 0)
        this->action->setGoBack(val);
}

void Etape::addVoisin(int newVoisinIndex, bool autreSens)
{
    this->addVoisin(get(newVoisinIndex), autreSens);
}

void Etape::addVoisins(int newVoisinIndex)
{
    this->addVoisin(get(newVoisinIndex), true);
}
void Etape::addVoisins(int newVoisinIndex, int newVoisinIndex2)
{
    this->addVoisin(get(newVoisinIndex), true);
    this->addVoisin(get(newVoisinIndex2), true);
}
void Etape::addVoisins(int newVoisinIndex, int newVoisinIndex2, int newVoisinIndex3)
{
    this->addVoisin(get(newVoisinIndex), true);
    this->addVoisin(get(newVoisinIndex2), true);
    this->addVoisin(get(newVoisinIndex3), true);
}
void Etape::addVoisins(int newVoisinIndex,
                       int newVoisinIndex2,
                       int newVoisinIndex3,
                       int newVoisinIndex4)
{
    this->addVoisin(get(newVoisinIndex), true);
    this->addVoisin(get(newVoisinIndex2), true);
    this->addVoisin(get(newVoisinIndex3), true);
    this->addVoisin(get(newVoisinIndex4), true);
}
void Etape::addVoisins(int newVoisinIndex,
                       int newVoisinIndex2,
                       int newVoisinIndex3,
                       int newVoisinIndex4,
                       int newVoisinIndex5)
{
    this->addVoisin(get(newVoisinIndex), true);
    this->addVoisin(get(newVoisinIndex2), true);
    this->addVoisin(get(newVoisinIndex3), true);
    this->addVoisin(get(newVoisinIndex4), true);
    this->addVoisin(get(newVoisinIndex5), true);
}

void Etape::addVoisins(int newVoisinIndex,
                       int newVoisinIndex2,
                       int newVoisinIndex3,
                       int newVoisinIndex4,
                       int newVoisinIndex5,
                       int newVoisinIndex6)
{
    this->addVoisin(get(newVoisinIndex), true);
    this->addVoisin(get(newVoisinIndex2), true);
    this->addVoisin(get(newVoisinIndex3), true);
    this->addVoisin(get(newVoisinIndex4), true);
    this->addVoisin(get(newVoisinIndex5), true);
    this->addVoisin(get(newVoisinIndex6), true);
}

void Etape::setNumeroEtapeFinAction(int newNumeroEtapeFinAction)
{
    this->numeroEtapeFinAction = newNumeroEtapeFinAction;
}

int Etape::getNumeroEtapeFinAction()
{
    return this->numeroEtapeFinAction;
}

std::vector<Etape*>& Etape::initTableauEtapeTotal(int number)
{
    tableauEtapesTotal.resize(number, nullptr);

    return tableauEtapesTotal;
}

Etape* Etape::get(int index)
{
    if (tableauEtapesTotal[index] == 0)
    {
        /*new Etape(index);*/
    }
    return tableauEtapesTotal[index];
}

std::vector<Etape*>& Etape::getTableauEtapesTotal()
{
    return tableauEtapesTotal;
}
