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
        ButtonLeft              --> animation (switch)

    Match-load:
        ButtonUp                --> flywheel forward (switch)
        ButtonDown              --> flywheel reverse (switch)
        ButtonY                 --> anchor (switch)

    Endgame:
        ButtonB                 --> lift / elevation / climbing (switch)

    Driving:
        ButtonR1                --> intake (hold)
        ButtonR2                --> intake (hold)
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

#include "Mechanics/botDrive.h"
#include "Mechanics/botFlywheel.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botLift.h"
#include "Mechanics/botWings.h"

#include "Utilities/fieldInfo.h"

#include "Videos/brainVideos.h"

// Competition instance (global)
competition Competition;

// Variables
// Drive info
double botX = 5 * field::tileLengthIn * 2.54, botY = 0.5 * field::tileLengthIn * 2.54, botAngle = 180;
double motSpeedRpm, motAimSpeedRpm = 0;

// Global variables
int playingVideoId = 0;

/******************* -------------- *******************/
/*                   Pre-Autonomous                   */
/******************* -------------- *******************/

void pre_auton(void) {
    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();

    // Activites before the competition starts
    // Flywheel task
    task flywheelTask([] () -> int { flywheelThread(); return 1; });
    // Intake task
    task intakeTask([] () -> int { intakeThread(); return 1; });
    // Controller task
    task rum([] () -> int { preautonControllerThread(); return 1; });
    // Pre-auton
    runPreauton();
    // Stopping brake-types
    preautonDrive();
}

/******************* ---------- *******************/
/*                   Autonomous                   */
/******************* ---------- *******************/

void autonomous(void) {
    // Start autonomous
    timer benchmark;

    // Reset
    resetLift();

    // Autonomous user code
    runAutonomous();

    printf("Time spent: %.3f s\n", benchmark.value());
}

void userRunAutonomous() {
    // Wait until sensors are initialized
    task::sleep(100);
    while (!initComponentFinished) {
        task::sleep(10);
    }

    autonomous();
}

/******************* ------------ *******************/
/*                   User Control                   */
/******************* ------------ *******************/

void usercontrol(void) {
    // userRunAutonomous();

    // Keybinds
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
