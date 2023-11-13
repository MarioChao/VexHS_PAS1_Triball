/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       User                                                      */
/*    Created:      Tue Aug 29 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

/******************* -------- *******************/
/*                   Keybinds                   */
/******************* -------- *******************/
/*
    Cosmetic:
        ButtonDown              --> animation (switch)

    Matchload:
        ButtonUp                --> lift (switch)
        ButtonY                 --> anchor (switch)
        ButtonR2                --> flywheel speed (switch)

    Endgame:
        ButtonR1 & ButtonR2     --> elevation / climbing (switch)
        ButtonUp (Controller2)  --> elevation / climbing (switch)

    Driving:
        ButtonR1                --> intake (hold)
        ButtonL1                --> wings (switch)

    Driving Mode - Arcade drive two stick:
        Axis3                   --> arcade forward (hold)
        Axis1                   --> arcade turn (hold)

    Driving Mode - Arcade drive one stick:
        Axis3                   --> arcade forward (hold)
        Axis4                   --> arcade turn (hold)

    Driving Mode - Mario drive:
        ButtonA                 --> mario forward (hold)
        ButtonB                 --> mario backward (hold)
        Axis4                   --> mario turn (hold)

*/

/******************* -------- *******************/
/*                   main.cpp                   */
/******************* -------- *******************/

#include "main.h"
#include "preauton.h"
#include "Autonomous/auton.h"

#include "Mechanics/botAnchor.h"
#include "Mechanics/botDrive.h"
#include "Mechanics/botFlywheel.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botLift.h"
#include "Mechanics/botWings.h"

#include "Videos/brainVideos.h"

// Competition instance (global)
competition Competition;

// Variables
// Drive info
double botX, botY, botAngle;
double motSpeedRpm, motAimSpeedRpm = 0;

// Global variables
int playingVideoId = 0;
bool canControlIntake = true;

/******************* -------------- *******************/
/*                   Pre-Autonomous                   */
/******************* -------------- *******************/

void pre_auton(void) {
    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();

    // Activites before the competition starts
    // eeecegg
    task rum([] () -> int { Controller1.rumble(".--.-- -"); return 1; });
    // Flywheel task
    task flywheelTask([] () -> int { flywheelThread(); return 1; });
    // Elevation task
    task elevationTask([] () -> int { elevationThread(); return 1; });
    // Pre-auton
    runPreauton();
    // Stopping brake-types
    preautonLift();
    preautonDrive();
}

/******************* ---------- *******************/
/*                   Autonomous                   */
/******************* ---------- *******************/

void autonomous(void) {
    timer benchmark;

    // Reset
    resetLift();

    // Autonomous user code
    runAutonomous();

    printf("Time spent: %.3f s\n", benchmark.value());
}

/******************* ------------ *******************/
/*                   User Control                   */
/******************* ------------ *******************/

void usercontrol(void) {
    // Keybinds
    keybindAnchor();
    keybindDrive();
    keybindFlywheel();
    keybindLift();
    keybindWings();
    keybindVideos();

    // Reset
    resetLift();

    while (1) {
        // Joystick feedback
        // Drive
        controlDrive();
        // Intake
        controlIntake();
        // Elevation
        controlElevation();

        // Short delay
        task::sleep(20);
    }
}

int main() {
    // Callbacks for auton & driver
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Pre-Autonomous
    pre_auton();

    // Prevent main from exiting
    while (true) {
        task::sleep(100);
    }

}
