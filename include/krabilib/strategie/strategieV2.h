#ifndef STRATEGIEV2_H_INCLUDED
#define STRATEGIEV2_H_INCLUDED

#include "krabilib/asservissement/command.h"
#include "krabilib/strategie/ramasserVerreV2.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/sharpSensor.h"
#include "krabilib/strategie/ultrasoundSensor.h"
#include "krabilib/strategie/strategiev3.h"
//#include "krabilib/strategie/canonFilet.h"

#ifndef ROBOTHW
#include "krabilib/strategie/sensors.h"
#include <QPainter>
#else
#include "krabilib/strategie/tourelle.h"
#endif

class StrategieV2
{
    public:
        StrategieV2(bool yellow = false);
        virtual ~StrategieV2();

        static void update();
        static Command* setCurrentGoal(Position goal, bool goBack = false, float maxSpeed = VITESSE_LINEAIRE_MAX, Angle precisionAngle = -100.00, float stopAtDistance = 0.f);
        static Command* setCurrentGoalSmooth(Position goal, Position nextGoal, float smoothFactor = 100., bool goBack = false, float maxSpeed = VITESSE_LINEAIRE_MAX,
                                             Angle precisionAngle = -100.00);
        static Command* setCurrentGoal(Position goal, Position center, float vitesse, bool goBack = false, Angle precisionAngle = -100.00);
        static void stop();
        static Command *lookAt(Position pos, float maxSpeed = VITESSE_ANGULAIRE_MAX);
        static Command* lookAt(Angle a, float maxSpeed = VITESSE_ANGULAIRE_MAX);
        static void addTemporaryAction(MediumLevelAction* action, bool stopAfter = false);
        static void gatherGlass();
        static void setJustAvoided(bool value);
        static bool getJustAvoided();
        static bool willCollide();
        static bool isYellow();
        static void setYellow(bool yellow);
        static void emptySharpsToCheck();
        static void setEnTrainDeRecalibrer(bool recalibre);
        static void enableSharp(SharpSensor::SharpName name);
        static void enableSharpsGroup(bool front);
        static void setCommand(Command* command);
        static bool sharpDetects(SharpSensor::SharpName name);
        static void setTourneSurSoiMeme(bool tourne);
        static long getTimeSpent();
        static void resetTime();
        static bool *getSharpsToCheck();
        static SharpSensor** getSensors();

#ifndef ROBOTHW
static void paint(QPainter* p);
#endif
        static bool somethingDetected;

    private:
        static bool yellow;
        static int updateCount;
        static StrategieV2* strategie;
        static Command* currentCommand;
        static MediumLevelAction* currentAction;
        static MediumLevelAction* actionsToDo[32];
        static MediumLevelAction* delayedActions[10];
        static int numberOfActionsDelayed;
        static int actionsCount;
        static bool hasToGoBase;
        static bool mustDeleteAction, hasToStopAfterAction;
        static int glassGathered;
        static int timeSinceLastRecalibration;
        static bool hasJustAvoided;
        static SharpSensor** sharps;
        static UltrasoundSensor* ultrasoundSensor;
        static Position positionsDeSecours[4];
        static int robotBloque;
        static bool enTrainDeRecalibrerOdometrie;
        static MediumLevelAction* evitement;
        static int timer;
        static bool sharpsToCheck[SharpSensor::END_SHARP_NAME];
        static bool tourneSurSoiMeme;
        static int timeToRestart;
        #ifdef ROBOTHW
        //static Tourelle* tourelle;
          //  static int hysteresisTourelle;
        #endif
};

#endif // STRATEGIEV2_H_INCLUDED
