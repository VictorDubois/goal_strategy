#ifndef VECTOR_H
#define VECTOR_H

/** @brief Classe permettant de stocker dans un vecteur un ensemble d'objet quelconque du même type T */
template<typename T>
class vector
{
    public:
        /** @brief Constructeur de base d'un vecteur */
        vector();
        /** @brief Constructeur d'un vecteur avec un espace mémoire réservé *
        *   @param allocatedSize Taille du vecteur à réserver en mémoire */
        vector(int allocatedSize);
        /** @brief Destructeur de la classe */
        virtual ~vector();

        /** @brief Permet d'ajouter un élément à la fin du tableau *
        *   @param value Élement à ajouter */
        void push_back(T value);
        /** @brief Pour retourner un élément précis du vecteur *
        *   @param i Case de l'élément à retourner */
        T operator[](int i);
        /** @brief Permet de réduire la taille du tableau en mémoire à la taille réél dont a besoin le vecteur pour stocker toutes les données */
        void resize();
        /** @brief Accesseur de size */
        int getSize();
        /** @brief Réserver une certaine place en mémoire avant de travailler sur le vecteur *
        *   @param taille Taille à réserver en mémoire */
        void reserve(int taille);
        /** @brief Permet de réinitialiser le vector (size = 0) */
        void reset();

    protected:
    private:
    /** @brief Tableau de taille allocatedSize contenant size donnée */
    T* table;
    /** @brief Nombre de données en mémoire */
    int size;
    /** @brief Taille du talbeau alloué en mémoire */
    int allocatedSize;
};


template<typename T>
vector<T>::vector()
{
    allocatedSize = 0;
    size = 0;
    table = 0x0;
}

template<typename T>
vector<T>::vector(int allocatedSize)
{
    this->allocatedSize = allocatedSize;
    table = new T[allocatedSize];
    size = 0;
}

template<typename T>
vector<T>::~vector()
{
    delete[] table;
}

template<typename T>
void vector<T>::push_back(T value)
{
    if (size < allocatedSize)
    {
        table[size] = value;
        size++;
    }
    else //table trop petite
    {
        T* oldTable = table;
        allocatedSize++;
        table = new T[allocatedSize];
        for (int i =0; i< size; i++)
            table[i] = oldTable[i];
        delete[] oldTable;
        table[size] = value;
        size++;
    }
}

template<typename T>
T vector<T>::operator[](int i)
{
    if (i<size)
        return table[i];
    else
        return table[-1]; // Pour obtenir une erreur
}

template<typename T>
void vector<T>::resize()
{
    if (size != allocatedSize)
    {

        T* oldTable = table;
        allocatedSize = size;
        if (allocatedSize != 0)
            table = new T[allocatedSize];
        else
            table = 0x0;
        for (int i =0; i< size; i++)
            table[i] = oldTable[i];
        delete[] oldTable;
    }
}

template<typename T>
int vector<T>::getSize()
{
    return size;
}

template<typename T>
void vector<T>::reserve(int taille)
{
    if (taille > allocatedSize)
    {
        T* oldTable = table;
        allocatedSize = taille;
        table = new T[allocatedSize];
        for (int i =0; i< size; i++)
            table[i] = oldTable[i];
        delete[] oldTable;
    }
}

template<typename T>
void vector<T>::reset()
{
    size = 0;
}

#endif // VECTOR_H
