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
        ButtonX                 --> switch driving mode

    Endgame:
        ButtonR2                 --> lift / elevation (switch)

    Driving:
        ButtonR1                --> intake (hold) (*ROBOT HAS NO INTAKE)
        ButtonL1                --> front wings (switch)
        ButtonL2                --> both back wings (switch)

    Driving Mode - Arcade drive two stick (also controlled turn):
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
#include "Mechanics/botLift.h"
#include "Mechanics/botPuncher.h"
#include "Mechanics/botWings.h"

#include "Utilities/fieldInfo.h"
#include "Utilities/debugFunctions.h"

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
    // Tasks
    task puncherTask([] () -> int { puncherThread(); return 1; });
    task rum([] () -> int { preautonControllerThread(); return 1; });
    /*task printEncoder([] () -> int {
        while (true) {
            printf("Encoder rotation: %.3f\n", LookEncoder.rotation(rev));
            task::sleep(30);
        }
        return 1;
    });*/

    // Stopping brake-types
    preautonDrive();

    // Initialize sensors & components
    // Pre-auton
    runPreauton();
    
    // Show auton state
    showAutonRunType();
}

/******************* ---------- *******************/
/*                   Autonomous                   */
/******************* ---------- *******************/

void autonomous(void) {
    // Start autonomous
    timer benchmark;

    // Switch to a random video
    task switchVideo([] () -> int {
        srand(Brain.Timer.systemHighResolution());
        switchVideoState(rand() % 3 + 1);
        return 1;
    });

    // Reset puncher
    resetPuncherCall();

    // Autonomous user code
    runAutonomous();

    printf("Time spent: %.3f s\n", benchmark.value());
}

/// @brief A function for testing autonomous directly in usercontrol.
void userRunAutonomous() {
    // Wait until sensors are initialized
    task::sleep(1500);
    while (!isPreautonFinished()) {
        task::sleep(10);
    }

    // userRunAutonomous();
    autonomous();
}

/******************* ------------ *******************/
/*                   User Control                   */
/******************* ------------ *******************/

void usercontrol(void) {
    // Skills intro
    if (getAutonRunType() == autonomousType::DrivingSkills) {
        // autonSkillsIntro();
    } else if (isUserRunningAuton()) {
        userRunAutonomous();
    }

    // Keybinds
    keybindDrive();
    keybindLift();
    keybindWings();
    keybindVideos();

    // Reset
    setWingsState(false);
    resetPuncherCall();

    while (1) {
        // Joystick feedback
        // Drive
        controlDrive();
        // printf("Encoder: %.3f\n", LookEncoder.rotation(rev));

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