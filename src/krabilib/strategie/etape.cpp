#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/actionGoTo.h"
#include "krabilib/strategie/dijkstra.h"

#ifdef QTGUI
#include <QDebug>
#endif

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

    this->actionGoTo = new ActionGoTo(getPosition());

    if (idx != ETAPE_INVALID_IDX)
        tableauEtapesTotal[idx] = this;
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

int Etape::getDistance()
{
    return this->distance;
}

void Etape::setDistance(int distance)
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
    if (this->aEviter())
    {
        // On oublie qu'on a vu un robot
        this->setEtapeType((EtapeType)((int)this->getEtapeType() - ROBOT_VU_ICI));
    }
}

int* Etape::getDistances()
{
    return this->distances;
}

void Etape::setDistances(int* distances)
{
    this->distances = distances;
}

void Etape::computeChildDistances()
{
    if (nbChildren == 0)
        return;

    this->distances = new int[this->nbChildren];

    for (int i = 0; i < this->nbChildren; ++i)
    {
        this->distances[i] = Dijkstra::calculDistanceDirect(this->children[i], this);
    }
}

int* Etape::getEtapesLieesParFinirEtape()
{
    return this->numerosEtapesLieesParFinirEtape;
}

void Etape::setEtapesLieesParFinirEtape(int* numerosEtapesLieesParFinirEtape)
{
    this->numerosEtapesLieesParFinirEtape = numerosEtapesLieesParFinirEtape;
}

int Etape::getNombreEtapesLieesParFinirEtape()
{
    return this->nombreEtapesLieesParFinirEtape;
}

void Etape::finir(void)
{
    return;
}

void Etape::setScore(int score)
{
    this->score = score;
}

int Etape::getScore()
{
    return this->score;
}

void Etape::setAction(MediumLevelAction* action)
{
    this->action = action;

    if (this->position == Position())
    {
        this->position = action->getGoalPosition();
        this->type = action->getType();

        if (this->actionGoTo != 0)
            delete this->actionGoTo;
        this->actionGoTo = new ActionGoTo(getPosition());
    }
}

MediumLevelAction* Etape::getAction()
{
    return this->action;
}

ActionGoTo* Etape::getActionGoTo()
{
    return this->actionGoTo;
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
    if (this->actionGoTo != 0)
        this->actionGoTo->reset();
}

void Etape::setGoBack(bool val)
{
    if (this->action != 0)
        this->action->setGoBack(val);
    if (this->actionGoTo != 0)
        this->actionGoTo->setGoBack(val);
}

void Etape::addVoisin(int newVoisinIndex, bool autreSens)
{
    this->addVoisin(get(newVoisinIndex), autreSens);
}

void Etape::addVoisins()
{
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

#ifdef QTGUI
QString Etape::getNameType(EtapeType type)
{
    switch (type)
    {
    case POINT_PASSAGE:
        return "Passage";
    case DEPART:
        return "Départ";
        //#ifdef GOLDO2018
    case ABEILLE:
        return "Abeille";
        //#elif KRABI2016
    case CABINE:
        return "Cabine";
    case DUNE:
        return "Dune";
    case ZONE_CONSTRUCTION:
        return "Zone de construction";

    case CUBE_DEBUT:
        return "Pousser les cubes a l'init";
        //#endif
    default:
        return QString::number(type);
    }
}

QString Etape::getShortNameType(EtapeType type)
{
    switch (type)
    {
    case POINT_PASSAGE:
        return "";
    case DEPART:
        return "Start";
        //#ifdef GOLDO2018
    case ABEILLE:
        return "Abeille";
        //#elif KRABI2016
    case CABINE:
        return "Cabine";
    case DUNE:
        return "Dune";
    case ZONE_CONSTRUCTION:
        return "Z.C";

    case CUBE_DEBUT:
        return "Cube debut";
        //#endif
    default:
        return "";
    }
}

#endif
