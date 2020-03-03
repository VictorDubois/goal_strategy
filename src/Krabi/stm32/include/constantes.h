#ifndef CONSTANTES_H
#define CONSTANTES_H

// macros communes
#define MIN(a,b) ((a)>(b)?(b):(a))
#define MAX(a,b) ((a)<(b)?(b):(a))
#define DEGTORAD(a) (a/180.*M_PI)
#define ANGLETOPULSE(a) (a*500)/124+1500

#ifndef NULL
#define NULL 0
#endif

//#define ROBOTHW
//#define CAPTEURS déjà mis dans asservissement

// HCTL_HANDLER
#define HCTL_BCL 1875000

// MOTEUR
// #define NUM_TIMER_MOTEUR 4

#ifdef STM32F10X_MD // pour la STM32 H103 2014 v1 :
    static const int FREQUENCE_MOTEURS = 8000;//quartz de 8MHz;
    static const int FREQUENCE_COEUR = 72000000;//cadence à 72MHz;
#endif
#ifdef STM32F40_41xxx // pour la STM32 H405 2014 v1 :
    static const int FREQUENCE_MOTEURS = 8000;//quartz de 8MHz;
    static const int FREQUENCE_COEUR = 168000000;//cadence à 168MHz;
#endif
#ifdef STM32F10X_CL // pour la STM32 H107 2013 v2 :
    static const int FREQUENCE_MOTEURS = 25000;//quartz de 25MHz;
    static const int FREQUENCE_COEUR = 72000000;//cadence à 72MHz;
#endif

static const int FREQUENCE_PWM = 25000;//25kHz, fréquence élevée (>10kHz) sinon les moteurs aiment pas
// inaudible (>22kHz), et << à la fréqunce max des ponts en H (environ 100kHz) et <<< à la fréquence de coupure des optocoupleurs (10MHz)

#ifdef ROBOTHW
#ifdef STM32F40_41xxx //cf http://00xnor.blogspot.co.uk/2014/01/4-stm32-f4-general-purpose-timers-for.html
static const int PRESCALER_TIMER_MOTEUR = 1;//((APB1_FREQ = 42M) / (CNT_FREQ = 21M))-1
//Note : le 2x de la ligne ci-dessous ne devrait pas exister...
static const int PERIOD_TIMER_MOTEUR = (2*21000000/FREQUENCE_PWM)-1;//((CNT_FREQ) / (PWM_FREQ))-1)      / Autoreload reg value;
#else
static const int PRESCALER_TIMER_MOTEUR = 71;//72MHz, 0-indexé
static const int PERIOD_TIMER_MOTEUR = (FREQUENCE_COEUR/(FREQUENCE_MOTEURS*(PRESCALER_TIMER_MOTEUR+1)));
#endif
#define CLOCK_TIMER_MOTEUR Timer::Div1
#endif

// TRAPEZOIDALCOMMAND
#define DISTANCE_ARRET 100 // 15
#define ANGLE_ARRET 0.08 //0.08

// DEG TO RAD
#define M_PI 3.14159265358979323846

// Asservissement

#define SEUIL_COLISION 1
#define SEUIL_COLISION_ANG 0.002
#define MS_BETWEEN_UPDATE 5//5//25
#define CPT_BEFORE_RAZ 8500

// Capteurs

    /// @brief SEUIL_DETECTION Seuil de detection des capteurs sharp. Au dessus de cette valeur, un objet sera considéré comme présent devant le capteur.
#define SEUIL_DETECTION 2500 // 2000
    /// @brief COEFFICIENT_LIN_ULTRASON Coefficient multiplicateur utilisé pour transformé la valeur analogique rendu par le capteur ultrason en une distance en metres.
#define COEFFICIENT_LIN_ULTRASON 2.54/2.*1.674003419
    /// @brief NB_CAPTEUR_A_ADC Nombre de capteur nécéssitant d'utiliser un ADC
#if defined(STM32F10X_MD) || defined(STM32F40_41xxx)
    #define NB_CAPTEUR_A_ADC 8
#else
    #define NB_CAPTEUR_A_ADC 10
#endif

// Servomoteurs

    /// @brief BALAI_G_CLOSED Angle du servomoteur pour lequel le balai gauche est fermé.
#define BALAI_D_CLOSED 110
    /// @brief BALAI_D_CLOSED Angle du servomoteur pour lequel le balai droit est fermé.
#define BALAI_G_CLOSED -150
    /// @brief BALAI_G_OPEN Angle du servomoteur pour lequel le balai gauche est ouvert.
#define BALAI_D_OPEN -150
    /// @brief BALAI_D_OPEN Angle du servomoteur pour lequel le balai droit est ouvert.
#define BALAI_G_OPEN 110
    /// @brief RATEAU_HAUT Angle du servomoteur pour lequel le rateau est en haut.
#define RATEAU_HAUT -270
    /// @brief RATEAU_BAS Angle du servomoteur pour lequel le rateau est en haut.
#define RATEAU_BAS -180//-200

// Command

// NE PAS TOUCHER !!!!
#ifdef RRD2
    #define VITESSE_LINEAIRE_MAX 0.3//1
    #define VITESSE_ANGULAIRE_MAX 0.005//0.010
    #define ACCELERATION_LINEAIRE_MAX 0.01//0.025
    #define DECELERATION_LINEAIRE_MAX 0.01//0.020
    #define ACCELERATION_ANGULAIRE_MAX 0.0001//0.00025

    // ODOMETRIE
    #define TAILLE_ENTRAXE 205.00
    #define RAYON_ROUE 65.00/2.0
    #define NBR_TICK_PAR_TOUR 848

    // ASSERVISSEMENT
    // PIDFILTERANGLE
    #define FILTER_ANGLE_KP 20
    #define FILTER_ANGLE_KI 2
    #define FILTER_ANGLE_KD 0
    #define FILTER_ANGLE_COEF 1.0


    // PIDFILTERLINEAIRE
    #define FILTER_LINEAIRE_KP 1//0.12
    #define FILTER_LINEAIRE_KI 0.12
    #define FILTER_LINEAIRE_KD 0.0
    #define FILTER_LINEAIRE_COEF 1.0

    // Detection Bloquage
    #define ACCELER_LIN_MINIMALE_DETECTION_BLOQUAGE 0.45//Max : 1
    #define ACCELER_ANG_MINIMALE_DETECTION_BLOQUAGE 0.45//Max : 1
    #define DISTANCE_MINIMALE_DETECTION_BLOQUAGE 5//Nombre de mm parcouru en 1 seconde
    #define ANGLE_MINIMAL_DETECTION_BLOQUAGE 2// Angle en ° parcouru en 1 seconde
#elif defined(STM32F40_41xxx) || defined(STM32F10X_MD) // Krabi Jr - STM32 H405 / STM32 H103
    #define VITESSE_LINEAIRE_MAX 1.0
    #define VITESSE_ANGULAIRE_MAX 0.010
    #define ACCELERATION_LINEAIRE_MAX 0.025
    #define DECELERATION_LINEAIRE_MAX 0.020
    #define ACCELERATION_ANGULAIRE_MAX 0.00025

    // ODOMETRIE
    #define TAILLE_ENTRAXE 228.80
    #define RAYON_ROUE 64.00/2.0
    #define NBR_TICK_PAR_TOUR 8192

    // ASSERVISSEMENT
    // PIDFILTERANGLE
    #define FILTER_ANGLE_KP 17.
    #define FILTER_ANGLE_KI 0.7
    #define FILTER_ANGLE_KD 1.0
    #define FILTER_ANGLE_COEF 1.0


    // PIDFILTERLINEAIRE
    #define FILTER_LINEAIRE_KP 0.10
    #define FILTER_LINEAIRE_KI 0.0055
    #define FILTER_LINEAIRE_KD 0.3
    #define FILTER_LINEAIRE_COEF 1.0

    // Detection Bloquage
    #define ACCELER_LIN_MINIMALE_DETECTION_BLOQUAGE 0.9//Max : 1
    #define ACCELER_ANG_MINIMALE_DETECTION_BLOQUAGE 0.9//Max : 1
    #define DISTANCE_MINIMALE_DETECTION_BLOQUAGE 5//Nombre de mm parcouru en 1 seconde
    #define ANGLE_MINIMAL_DETECTION_BLOQUAGE 2// Angle en ° parcouru en 1 seconde

#else // Krabi - STM32 H107
    #define VITESSE_LINEAIRE_MAX 2.0 //4.0
    #define VITESSE_ANGULAIRE_MAX 0.005 //0.025//0.05//0.025
    #define ACCELERATION_LINEAIRE_MAX 0.025 //0.045//0.045//0.08//0.15//0.15//0.1//0.05
    #define DECELERATION_LINEAIRE_MAX 0.020 //0.02//0.08//0.15//0.15//0.1//0.05
    #define ACCELERATION_ANGULAIRE_MAX 0.00025 // 0.00035//(2*M_PI/20000.0) //(2*M_PI/5000.0)

    // ODOMETRIE
    #define TAILLE_ENTRAXE 345.50//345.70
    #define RAYON_ROUE 32.00//31.30
    #define NBR_TICK_PAR_TOUR 8192

    // ASSERVISSEMENT
    // PIDFILTERANGLE
    #define FILTER_ANGLE_KP 20.0
    #define FILTER_ANGLE_KI 0.17
    #define FILTER_ANGLE_KD 0.1
    #define FILTER_ANGLE_COEF 1.0//0.89

    // PIDFILTERLINEAIRE
    #define FILTER_LINEAIRE_KP 0.25//0.20//0.2//0.12
    #define FILTER_LINEAIRE_KI 0.0075//0.0005//0.005//0.005
    #define FILTER_LINEAIRE_KD 0.1
    /*#define FILTER_LINEAIRE_KP 0.2//0.12
    #define FILTER_LINEAIRE_KI 0.005//0.005//0.005
    #define FILTER_LINEAIRE_KD 0.7//0.7*/
    #define FILTER_LINEAIRE_COEF 1.0//0.89

    // Detection Bloquage
    #define ACCELER_LIN_MINIMALE_DETECTION_BLOQUAGE 0.9//Max : 1
    #define ACCELER_ANG_MINIMALE_DETECTION_BLOQUAGE 0.9//Max : 1
    #define DISTANCE_MINIMALE_DETECTION_BLOQUAGE 5//Nombre de mm parcouru en 1 seconde
    #define ANGLE_MINIMAL_DETECTION_BLOQUAGE 2// Angle en ° parcouru en 1 seconde
#endif


// ROUE
#define RAPPORT_OK 0.05//0.05
#define RAPPORT_SUPPLEMENTAIRE 0.08

// SERVO
/// @brief NUMERO_TIMER_SERVO Numéro du Timer (TIM) sur lequel sont les servos
#ifdef STM32F40_41xxx//STM32F10X_MD // pour la STM32 H103 2014 v1 :
    #define NUMERO_TIMER_SERVO 3
#endif
#ifdef STM32F10X_CL // pour la STM32 H107 2013 v2 :
    #define NUMERO_TIMER_SERVO 1
#endif

//#define NUMERO_TIMER_SERVO 3
/// @brief PERIOD_TIMER_SERVO Période du timer.
//#define PERIOD_TIMER_SERVO 20000
/// @brief PRESCALER_TIMER_SERVO Facteur de division de l’horloge : @f$ fr\acute{e}quence du timer = \frac{72 MHz}{ timClockDivision (timPrescaler + 1)} @f$ .
//#define PRESCALER_TIMER_SERVO 71
/// @brief CLOCK_TIMER_SERVO : division d’horloge supplémentaire, voire Timer::ClkDivision.
#define CLOCK_TIMER_SERVO Timer::Div1
//#define FREQUENCE_SERVO 1e6

#ifdef ROBOTHW
// les servo analogique fonctionnent à 50 Hz
static const int FREQUENCE_IMPULSION_SERVO = 50;
// mettre une haute valeur pour avoir plus de précision sur le rapport cyclique (doit etre un diviseur de la fréquence d'horloge du timer)
static const int FREQUENCE_TIMER_SERVO = 1000000;
// (fréquence de l'horloge du timer)/(fréquence du timer) - 1
static const int PRESCALER_TIMER_SERVO = FREQUENCE_COEUR/FREQUENCE_TIMER_SERVO - 1;
// correspond au nombre de valeurs possibles pour le rapport cyclique
// sachant que les servo analogiques ne fonctionnent que sur une plage de rapport cyclique bien précise
// cela permet de connaitre la précision angulaire théorique des servo
// précision en degré = PERIOD_TIMER_SERVO*(RC max - RC min)/180
static const int PERIOD_TIMER_SERVO = FREQUENCE_TIMER_SERVO/FREQUENCE_IMPULSION_SERVO;
#endif
// STRATEGIE
#define INSTRUCTION_COLLISION 128
#define POS_DEPART_X 260
#define POS_DEPART_Y 250
#define RAYON_ROBOT 250
#define ANGLE_DEPART 0
// COMMAND_GO_TO
#define PATH_LENGTH 800

#endif // CONSTANTES_H
