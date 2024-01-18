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
    autonomousType auton_runType = autonomousType::NearAWP;
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
    // setFlywheelSpeedRpm(460);
    // Push the two alliance triballs
    driveAndTurnDistanceTiles(-2.0, 180, 75.0, 500.0, defaultMoveTilesErrorRange, 1.4);
    setIntakeState(0);

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
    // setFlywheelSpeedRpm(480);
    while (duration.value() < 23.0) {
        task::sleep(10);
    }

    // For practicing match-loading
    // while (duration.value() < 60.0) {
    //     task::sleep(10);
    // }

    // Finish matchload
    // setFlywheelSpeedRpm(0);
    
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
        // turnToAngle(90);
        // turnToAngle(-90);
        // turnToAngle(180);
        // turnToAngle(-180);
        // turnToAngle(0);
    }

    /// @brief Run the 15-seconds near-side AWP autonomous.
    void runAutonNearAWP() {
        // For PASVEX's Robot
        timer autontimer;
        setRotation(180.0);
        
        // Push middle balls over the middle barrier
        driveAndTurnDistanceTiles(-1.82, 200.0, 800.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        setRightWingState(true);
        turnToAngleVelocity(270.0, 60.0, -halfRobotLengthIn * 0.5, defaultTurnAngleErrorRange, 1.3);
        setIntakeState(0);
        driveAndTurnDistanceTiles(-1.60, 270.0, 100.0, 200.0, defaultMoveTilesErrorRange, 1.3);
        setIntakeState(1);
        setRightWingState(false, 0.3);

        // Drive to match load zone
        driveAndTurnDistanceTiles(0.30, 270.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        turnToAngle(215.0, -halfRobotLengthIn, defaultTurnAngleErrorRange, 1.5);
        driveAndTurnDistanceTiles(2.00, 235.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
        // Swing the ball out
        setRightWingState(true);
        turnToAngle(135, -halfRobotLengthIn * 1.2, defaultTurnAngleErrorRange, 1.0);
        driveAndTurnDistanceTiles(0.70, 45.0, 60.0, 400.0, defaultMoveTilesErrorRange, 1.5);
        setRightWingState(false);
        task::sleep(500);
        // turnToAngleVelocity(310, 40.0, 0.0, defaultTurnAngleErrorRange, 1.5);
        turnToAngle(290, 0.0, defaultTurnAngleErrorRange, 1.5);
        
        // Prepare to push balls over to the offensive zone
        task::sleep(500);
        driveAndTurnDistanceTiles(-0.80, 270.0, 30.0, 8.0, defaultMoveTilesErrorRange, 1.7);

        // Idle until 11th second of autonomous
        while (autontimer.value() < 11.0) {
            task::sleep(20);
        }

        // Push the preload and former-corner ball over to the offensive zone
        driveAndTurnDistanceTiles(-1.00, 270.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);

        // Touch the elevation bar
        task::sleep(500);
        driveAndTurnDistanceTiles(-1.00, 270.0, 10.0, 10.0, defaultMoveTilesErrorRange, 2.5);
        // Robot is touching the elevation bar
    }

    /// @brief Run the 15-seconds near-side AWP-safe autonomous.
    void runAutonNearAWPSafe() {

    }

    /// @brief Run the 15-seconds far-side AWP autonomous.
    void runAutonFarAWP() {
        setRotation(0.0);

        // Go near the center
        driveAndTurnDistanceTiles(1.80, -5.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.7);
        // Face goal and release preload ball
        turnToAngle(45.0, halfRobotLengthIn, defaultTurnAngleErrorRange, 1.0);
        task::sleep(200);
        setIntakeState(0);

        // Face center-down ball
        driveAndTurnDistanceTiles(-0.80, 45.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        turnToAngle(-90.0, 0.0, defaultTurnAngleErrorRange, 1.5);
        // Intake ball
        driveAndTurnDistanceTiles(1.00, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        setIntakeState(1);
        task::sleep(100);
        // Face goal, drive, and release ball
        turnToAngle(60.0, 0.0, defaultTurnAngleErrorRange, 0.8);
        driveAndTurnDistanceTiles(0.90, 60.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        task::sleep(200);
        setIntakeState(0);
        driveAndTurnDistanceTiles(-0.60, 60.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);

        // Face center-middle ball
        turnToAngle(-40.0, 0.0, defaultTurnAngleErrorRange, 0.7);
        // Intake ball
        driveAndTurnDistanceTiles(0.70, -35.0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        setIntakeState(1);
        // Push balls through left-side of the goal
        turnToAngle(-87.0, 0.0, defaultTurnAngleErrorRange, 0.7);
        setWingsState(true);
        task::sleep(200);
        driveAndTurnDistanceTiles(-1.90, -90.0, 100.0, 300.0, defaultMoveTilesErrorRange, 2.0);
        setWingsState(false);
        // Release loaded ball into the goal
        turnToAngle(90.0, 0.0, defaultTurnAngleErrorRange, 0.7);
        setIntakeState(0);
        driveAndTurnDistanceTiles(-0.30, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        setIntakeState(1);
        driveAndTurnDistanceTiles(-0.70, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        
        // Intake ball below elevation bar (not included)

        // Go to matchload zone
        turnToAngle(180.0, 0.0, defaultTurnAngleErrorRange, 0.7);
        driveAndTurnDistanceTiles(1.80, 150.0, 100.0, 20.0, defaultMoveTilesErrorRange, 1.7);
        // Swing the corner ball out
        setRightWingState(true);
        turnToAngle(0.0, -halfRobotLengthIn, defaultTurnAngleErrorRange, 1.7);
        setRightWingState(false);
        // Push ball through the bottom-side of the goal
        turnToAngle(-165.0, 0.0, defaultTurnAngleErrorRange, 1.0);
        driveAndTurnDistanceTiles(-1.50, -180.0, 100.0, 1000.0, defaultMoveTilesErrorRange, 1.3);
        driveAndTurnDistanceTiles(0.70, -180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
    }

    /// @brief Run the 15-seconds far-side Eliminations autonomous.
    void runAutonFarElim() {
        
    }

    /// @brief Run the skills autonomous.
    void runAutonSkills() {
        autonSkillsIntro();

        // Push the triballs through path that goes below the red elevation-bar
        driveAndTurnDistanceTiles(-3.0, -80.0, 100.0, 300.0, defaultMoveTilesErrorRange, 2.2);
        setRightWingState(true);
        driveAndTurnDistanceTiles(-1.0, -80.0, 100.0, 300.0, defaultMoveTilesErrorRange, 1.5);

        // Score triballs through bottom-side of the goal
        turnToAngle(-100, 0, defaultTurnAngleErrorRange, 1.0);
        setRightWingState(false, 0.3);
        driveAndTurnDistanceTiles(-3.0, -180.0, 100.0, 60.0, defaultMoveTilesErrorRange, 1.2);

        // Push again through bottom-side of the goal
        task::sleep(300);
        driveAndTurnDistanceTiles(0.7, -155, 100.0, 300.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-2.0, -180.0, 100.0, 700.0, defaultMoveTilesErrorRange, 0.8);
        
        // Score triballs through left-side of the goal
        task::sleep(300);
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
        driveAndTurnDistanceTiles(-2.5, -90, 130.0, 50.0, defaultMoveTilesErrorRange, 1.5);

        // Score triballs through top-side of the goal
        driveAndTurnDistanceTiles(1.7, 23.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        setWingsState(false);
        driveAndTurnDistanceTiles(1.5, -90.0, 300.0, 80.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-1.3, -40.0, 80.0, 300.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-2.0, 0.0, 70.0, 600.0, defaultMoveTilesErrorRange, 1.3);

        // Push again through top-side of the goal
        task::sleep(300);
        driveAndTurnDistanceTiles(0.7, -25.0, 100.0, 300.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-2.0, 0.0, 100.0, 700.0, defaultMoveTilesErrorRange, 0.8);

        // Elevation
        task::sleep(300);
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