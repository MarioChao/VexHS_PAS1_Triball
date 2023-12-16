#include "Autonomous/auton.h"
#include "Autonomous/autonFunctions.h"
#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"
#include "preauton.h"
#include "main.h"

namespace {
    using namespace auton;
    using namespace botinfo;
    using debug::printOnController;

    void autonTest();

    void runAutonNearAWP();
    void runAutonNearAWPSafe();
    void runAutonNearElim();
    void runAutonFarAWP();
    void runAutonFarElim();
    void runAutonSkills();

    bool userRunningAutonomous = false;
    autonomousType auton_runType = autonomousType::FarAWP;
    int auton_allianceId;
}

void setAutonRunType(int allianceId, autonomousType autonType) {
    switch (autonType) {
        case autonomousType::NearAWP:
            printOnController("Auton: NearAWP");
            printf("Nearaw\n");
            break;
        case autonomousType::NearAWPSafe:
            printOnController("Auton: NearSafe");
            printf("Nearaw Safe\n");
            break;
        case autonomousType::NearElim:
            printOnController("Auton: NearElim");
            printf("Nearel\n");
            break;
        case autonomousType::FarAWP:
            printOnController("Auton: FarAWP");
            printf("Faraw\n");
            break;
        case autonomousType::FarElim:
            printOnController("Auton: FarElim");
            printf("Farel\n");
            break;
        case autonomousType::AutonSkills:
            printOnController("Auton: Skills");
            printf("AuSk\n");
            break;
        case autonomousType::DrivingSkills:
            printOnController("Driving Skills");
            printf("DrSk\n");
            break;
        default:
            printOnController("Auton: None");
            printf("None\n");
            break;
    }
    auton_runType = autonType;
    auton_allianceId = allianceId;
}

void showAutonRunType() {
    setAutonRunType(auton_allianceId, auton_runType);
}

autonomousType getAutonRunType() {
    return auton_runType;
}

bool isUserRunningAuton() {
    return userRunningAutonomous;
}

void runAutonomous() {
    printf("Auton time!\n");
    userRunningAutonomous = false;
    switch (auton_runType) {
        case autonomousType::NearAWP:
            runAutonNearAWP();
            break;
        case autonomousType::NearAWPSafe:
            runAutonNearAWPSafe();
            break;
        case autonomousType::NearElim:
            // runAutonNearElim();
            break;
        case autonomousType::FarAWP:
            runAutonFarAWP();
            break;
        case autonomousType::FarElim:
            runAutonFarElim();
            break;
        case autonomousType::AutonSkills:
            runAutonSkills();
            break;
        case autonomousType::Test:
            autonTest();
        default:
            break;
    }
}

void autonSkillsIntro() {
    setRotation(135);

    // Prepare for matchload
    setFlywheelSpeedRpm(460);
    // Push the two alliance triballs
    driveAndTurnDistanceTiles(-2.0, 180, 75.0, 500.0, defaultMoveTilesErrorRange, 1.4);

    // Go to matchload position
    task::sleep(200);
    driveAndTurnDistanceTiles(1.2, 140.0, 60.0, 900.0, defaultMoveTilesErrorRange, 1.5);
    // Match load positioning
    turnToAngle(60, 0.0, defaultTurnAngleErrorRange, 1.0);
    driveAndTurnDistanceTiles(-0.5, 60, 25.0, 200.0, defaultMoveTilesErrorRange, 0.7);
    turnToAngleVelocity(74.0, 60.0, 0.0, defaultTurnAngleErrorRange, 1.5);

    // Match load 44 balls
    task::sleep(2000);
    timer duration;
    while (duration.value() < 10.0) {
        task::sleep(10);
    }
    setFlywheelSpeedRpm(480);
    while (duration.value() < 23.0) {
        task::sleep(10);
    }

    // For practicing match-loading
    // while (duration.value() < 60.0) {
    //     task::sleep(10);
    // }

    // Finish matchload
    setFlywheelSpeedRpm(0);
    
    // Back-side face the elevation bar
    turnToAngle(-60, 0.0, defaultTurnAngleErrorRange, 1.0);
}

namespace {
    void autonTest() {
        setRotation(0.0);
        driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
        driveAndTurnDistanceTiles(-2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
    }

    /// @brief Run the 15-seconds near-side AWP autonomous.
    void runAutonNearAWP() {
        // For PASVEX's Robot
        timer autontimer;
        setRotation(180.0);
        
        // Push middle balls over the middle barrier
        driveAndTurnDistanceTiles(-1.82, 195.0, 800.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        setIntakeState(0);
        setRightWingState(true);
        turnToAngleVelocity(270.0, 60.0, -halfRobotLengthIn * 0.5, defaultTurnAngleErrorRange, 1.3);
        driveAndTurnDistanceTiles(-1.3, 270.0, 100.0, 200.0, defaultMoveTilesErrorRange, 1.3);
        setRightWingState(false, 0.3);

        // Drive to match load zone
        driveAndTurnDistanceTiles(0.3, 270.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        turnToAngle(215.0, -halfRobotLengthIn, defaultTurnAngleErrorRange, 1.5);
        driveAndTurnDistanceTiles(2.13, 235.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
        // Face upward-parallel to the match load bar
        turnToAngle(315.0, halfRobotLengthIn * 1.25, defaultTurnAngleErrorRange, 1.5);
        // Swing the ball out
        setLeftWingState(true);
        turnToAngle(300, 0.0, defaultTurnAngleErrorRange, 1.0);
        driveAndTurnDistanceTiles(-0.7, 290.0, 60.0, 400.0, defaultMoveTilesErrorRange, 1.5);
        setLeftWingState(false);
        task::sleep(500);
        turnToAngleVelocity(310, 40.0, 0.0, defaultTurnAngleErrorRange, 1.5);
        
        // Prepare to push balls over to the offensive zone
        task::sleep(500);
        driveAndTurnDistanceTiles(-0.5, 270.0, 30.0, 8.0, defaultMoveTilesErrorRange, 1.7);
        turnToAngle(250.0, halfRobotLengthIn * 0.5, defaultTurnAngleErrorRange, 0.7);
        turnToAngle(270.0, -halfRobotLengthIn * 0.5, defaultTurnAngleErrorRange, 0.7);

        // Idle until 11th second of autonomous
        while (autontimer.value() < 11.0) {
            task::sleep(20);
        }

        // Push the preload and former-corner ball over to the offensive zone
        driveAndTurnDistanceTiles(-0.8, 270.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);

        // Touch the elevation bar
        setRightWingState(true);
        driveAndTurnDistanceTiles(-0.8, 260.0, 20.0, 20.0, defaultMoveTilesErrorRange, 2.5);
        turnToAngle(300.0, -halfRobotLengthIn * 0.5, defaultTurnAngleErrorRange, 1.5);
        // Robot is touching the elevation bar
    }

    /// @brief Run the 15-seconds near-side AWP-safe autonomous.
    void runAutonNearAWPSafe() {
        // For PASVEX's Robot
        timer autontimer;
        setRotation(180.0);
        
        // Don't push middle balls over the middle barrier
        driveAndTurnDistanceTiles(-1.2, 195.0, 800.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        setIntakeState(0);

        // Drive to match load zone
        turnToAngle(195.0, 0, defaultTurnAngleErrorRange, 1.5);
        driveAndTurnDistanceTiles(0.9, 235.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
        // Face upward-parallel to the match load bar
        turnToAngle(315.0, halfRobotLengthIn * 1.25, defaultTurnAngleErrorRange, 1.5);
        // Swing the ball out
        setLeftWingState(true);
        turnToAngle(300, 0.0, defaultTurnAngleErrorRange, 1.0);
        driveAndTurnDistanceTiles(-0.7, 290.0, 60.0, 400.0, defaultMoveTilesErrorRange, 1.5);
        setLeftWingState(false);
        task::sleep(500);
        turnToAngleVelocity(310, 40.0, 0.0, defaultTurnAngleErrorRange, 1.5);
        
        // Prepare to push balls over to the offensive zone
        task::sleep(500);
        driveAndTurnDistanceTiles(-0.5, 270.0, 30.0, 8.0, defaultMoveTilesErrorRange, 1.7);
        turnToAngle(250.0, halfRobotLengthIn * 0.75, defaultTurnAngleErrorRange, 0.7);
        turnToAngle(270.0, -halfRobotLengthIn * 0.75, defaultTurnAngleErrorRange, 0.7);

        // Idle until 11th second of autonomous
        while (autontimer.value() < 11.0) {
            task::sleep(20);
        }

        // Push the preload and former-corner ball over to the offensive zone
        driveAndTurnDistanceTiles(-0.8, 270.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);

        // Touch the elevation bar
        setRightWingState(true);
        driveAndTurnDistanceTiles(-0.8, 250.0, 20.0, 100.0, defaultMoveTilesErrorRange, 2.5);
        turnToAngle(300.0, -halfRobotLengthIn * 0.5, defaultTurnAngleErrorRange, 1.5);
        // Robot is touching the elevation bar
    }

    /// @brief Run the 15-seconds far-side AWP autonomous.
    void runAutonFarAWP() {
        setRotation(-90.0);
        
        // Intake ball below elevation bar
        setIntakeState(1);
        driveAndTurnDistanceTiles(0.4, -85.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.6);

        // Go to match load zone while pushing preload ball
        driveAndTurnDistanceTiles(-1.6, -160.0, 60.0, 6.0, defaultMoveTilesErrorRange, 1.6);
        // Swing the corner ball out
        setLeftWingState(true);
        task::sleep(20);
        setLeftWingState(false, 0.5);
        driveAndTurnDistanceTiles(-0.5, -178.0, 50.0, 30.0, defaultMoveTilesErrorRange, 0.7);
        // Push two balls through the bottom-side of the goal
        turnToAngle(-160, 0.0, defaultTurnAngleErrorRange, 0.6);
        task::sleep(100);
        driveAndTurnDistanceTiles(-2.0, -180.0, 100.0, 1000.0, defaultMoveTilesErrorRange, 0.6);

        // Face and drive partially to center-down ball
        driveAndTurnDistanceTiles(1.0, -60.0, 100.0, 300.0, defaultMoveTilesErrorRange, 0.9);
        driveAndTurnDistanceTiles(1.0, -60.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // Face goal and out-take ball
        turnToAngle(49, 0.0, defaultTurnAngleErrorRange, 1.5);
        setIntakeState(-1);
        task::sleep(250);

        // Face center-down ball
        turnToAngle(-70.0, 0.0, defaultTurnAngleErrorRange, 1.5);
        // Intake ball
        setIntakeState(1);
        driveAndTurnDistanceTiles(0.87, -70.0, 100.0, 200.0, defaultMoveTilesErrorRange, 1.3);
        // Face goal, drive, and out-take ball
        turnToAngle(57.0, 0.0, defaultTurnAngleErrorRange, 0.8);
        driveAndTurnDistanceTiles(0.7, 57.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        setIntakeState(-1);
        task::sleep(250);

        // Face center-middle ball
        turnToAngle(-60.0, 0.0, defaultTurnAngleErrorRange, 0.7);
        // Intake ball
        setIntakeState(1);
        driveAndTurnDistanceTiles(0.85, -30.0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        // Push two balls through left-side of the goal
        turnToAngle(-85.0, 0.0, defaultTurnAngleErrorRange, 0.6);
        setWingsState(true);
        driveAndTurnDistanceTiles(-2.0, -90.0, 100.0, 300.0, defaultMoveTilesErrorRange, 0.75);
        // Push loaded ball into the goal
        driveAndTurnDistanceTiles(0.5, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        turnToAngle(90.0, 0.0, defaultTurnAngleErrorRange, 0.9);
        setWingsState(false);
        setIntakeState(-1);
        driveAndTurnDistanceTiles(1.5, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
        setIntakeState(0);
        
        // Drive and touch the elevation bar using wings
        setLeftWingState(true, 1.05);
        driveAndTurnDistanceTiles(-4.0, 0.0, 120.0, 90.0, defaultMoveTilesErrorRange, 2.5);
        // Robot is touching the elevation bar
    }

    /// @brief Run the 15-seconds far-side Eliminations autonomous.
    void runAutonFarElim() {
        setRotation(-36);

        // Expand right wing to push preload triball
        setRightWingState(true);
        task retractWing([] () -> int {
            task::sleep(150);
            setRightWingState(false);
            return 1;
        });

        // Score the middle balls
        driveAndTurnDistanceTiles(sqrt(pow(2.35, 2) + pow(2.35, 2)), -36, 100.0, 1000.0, 0.05, 2.3);
        // Intake middle ball
        setIntakeState(true);
        turnToAngle(-65, -halfRobotLengthIn, 3.0, 0.7); // Face up-left (more left)
        // Score the balls
        turnToAngle(90, 0, 3.0, 1.0); // Face the goal
        setIntakeState(false);
        setWingsState(true);
        driveAndTurnDistanceTiles(2.5, 90, 100.0, 200.0, 0.05, 1.5);
        setWingsState(false);

        // Intake a triball from middle
        driveDistanceTiles(-0.5, 100.0, 0.05, 0.5);
        turnToAngle(234, 0, 3.0, 1.0); // Face the triball
        // Intake ball
        setIntakeState(true);
        driveAndTurnDistanceTiles(sqrt(pow(1.32, 2) + pow(1.32, 2)), 234, 100.0, 100.0, 0.05, 1.3);
        turnToAngle(260, halfRobotLengthIn, 3.0, 0.7); // Face the down-left (more left)

        // Go to match load zone
        turnToAngle(310, -halfRobotLengthIn * 1.5, 3.0, 1.0); // Face the matchload zone
        // Move toward match load zone
        setIntakeState(true);
        driveAndTurnDistanceTiles(-sqrt(pow(1.65, 2) + pow(1.65, 2)), 310, 100.0, 100.0, 0.05, 2.0);

        // Push the corner triball out
        turnToAngle(405, -halfRobotLengthIn * 0.3, 4.0, 1.0); // Face parallel to the matchload zone
        // Outtake
        setIntakeState(-1);
        // Anchor swing the ball
        // setAnchorState(true);
        task::sleep(100);
        turnToAngle(345, -halfRobotLengthIn, 3.0, 1.0); // Face up-left (swing the ball out)
        // setAnchorState(false);
        turnToAngle(375, 0.0, 1.0, 0.5); // Unstuck the anchor
        setIntakeState(false);

        // Push the preload, former-corner, and released triballs into the goal
        // Push with back
        turnToAngle(210, 0, 3.0, 1.0);
        driveAndTurnDistanceTiles(-2.5, 180, 100.0, 200.0, 0.05, 0.7);
        driveAndTurnDistanceTiles(1.0, 205, 100.0, 190.0, 0.05, 1.0);
        driveAndTurnDistanceTiles(-2.0, 180, 100.0, 200.0, 0.05, 0.7);

        // Drive a little back
        driveDistanceTiles(0.3, 100.0, 0.05, 1.0);
    }

    /// @brief Run the skills autonomous.
    void runAutonSkills() {
        autonSkillsIntro();

        // Push the triballs through path that goes below the red elevation-bar
        driveAndTurnDistanceTiles(-3.0, -80.0, 100.0, 300.0, defaultMoveTilesErrorRange, 2.2);
        setRightWingState(true);
        driveAndTurnDistanceTiles(-1.2, -80.0, 100.0, 300.0, defaultMoveTilesErrorRange, 1.5);

        // Score triballs through bottom-side of the goal
        turnToAngle(-100, 0, defaultTurnAngleErrorRange, 1.0);
        setRightWingState(false, 0.3);
        driveAndTurnDistanceTiles(-3.0, -180.0, 100.0, 70.0, defaultMoveTilesErrorRange, 1.2);

        // Push again through bottom-side of the goal
        driveAndTurnDistanceTiles(1.0, -155, 100.0, 300.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-2.0, -180.0, 100.0, 700.0, defaultMoveTilesErrorRange, 1.2);
        
        // Score triballs through left-side of the goal
        task::sleep(500);
        driveAndTurnDistanceTiles(0.3, -180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        turnToAngle(-70, halfRobotLengthIn * 0.75, defaultTurnAngleErrorRange, 1.5);
        driveAndTurnDistanceTiles(2.0, -90, 90.0, 300.0, defaultMoveTilesErrorRange, 1.5);
        turnToAngle(-180, 0, defaultTurnAngleErrorRange, 1.5);
        setWingsState(true);
        driveAndTurnDistanceTiles(-2.5, -90.0, 100.0, 700.0, defaultMoveTilesErrorRange, 1.5);
        setWingsState(false, 0.3);

        // Score more triballs through left-side of the goal
        driveAndTurnDistanceTiles(1.7, -180, 1000.0, 30.0, defaultMoveTilesErrorRange, 1.5);
        setLeftWingState(true);
        driveAndTurnDistanceTiles(-2.2, -180, 60.0, 100.0, defaultMoveTilesErrorRange, 3.0);
        turnToAngleVelocity(-20, 60.0, -halfRobotLengthIn * 0.5, defaultTurnAngleErrorRange, 1.5);
        setRightWingState(true);
        driveAndTurnDistanceTiles(-2.5, -90, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);

        // Score triballs through top-side of the goal
        driveAndTurnDistanceTiles(1.7, 23.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        setWingsState(false);
        driveAndTurnDistanceTiles(1.5, -90.0, 300.0, 80.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-1.3, -40.0, 80.0, 300.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-2.0, 0.0, 70.0, 600.0, defaultMoveTilesErrorRange, 1.5);

        // Elevation
        task::sleep(500);
        driveAndTurnDistanceTiles(0.5, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // Prepare for elevation
        setLiftState(true);
        // Drive to elevation bar
        turnToAngle(-40, 0.0, defaultTurnAngleErrorRange, 1.5);
        driveAndTurnDistanceTiles(3.6, -90, 150.0, 120.0, defaultMoveTilesErrorRange, 3.0);
        // Clamp on the bar
        task::sleep(100);
        setLiftState(false);
    }
}