#include "Autonomous/auton.h"
#include "Autonomous/autonFunctions.h"
#include "Mechanics/botPuncher.h"
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
    void runAutonSkillsCrossBarrier();

    bool userRunningAutonomous = false;
    autonomousType auton_runType = autonomousType::AutonSkills;
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

namespace {
    void autonTest() {
        setRotation(0.0);
        // driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTiles(2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
        // driveAndTurnDistanceTiles(-2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);

        // driveAndTurnDistanceTilesMotionProfile(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTilesMotionProfile(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTilesMotionProfile(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTilesMotionProfile(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTilesMotionProfile(2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
        // driveAndTurnDistanceTilesMotionProfile(-2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);

        turnToAngle(90);
        turnToAngle(-90);
        turnToAngle(180);
        turnToAngle(-180);
        turnToAngle(0);
    }

    /// @brief Run the 15-seconds near-side AWP autonomous.
    void runAutonNearAWP() {
        // For PASVEX's Robot
        timer autontimer;
        setRotation(180.0);
        
        // Release triball in front of goal
        driveAndTurnDistanceTiles(-1.85, 200.0, 800.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        turnToAngle(275.0, -halfRobotLengthIn * 0.3, defaultTurnAngleErrorRange, 1.3);
        setIntakeState(0);
        turnToAngle(270.0, 0, defaultTurnAngleErrorRange, 0.7);
        // Push middle balls over the middle barrier
        setRightWingState(true);
        driveAndTurnDistanceTiles(-1.50, 270.0, 100.0, 200.0, defaultMoveTilesErrorRange, 1.3);
        // setRightWingState(false);

        // Drive to match load zone
        setRightWingState(false);
        turnToAngle(215.0, -halfRobotLengthIn, defaultTurnAngleErrorRange, 1.5);
        driveAndTurnDistanceTiles(2.00, 235.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
        // Swing the ball out
        setRightWingState(true);
        turnToAngle(135, -halfRobotLengthIn * 1.2, defaultTurnAngleErrorRange, 1.0);
        // driveAndTurnDistanceTiles(0.70, 45.0, 40.0, 400.0, defaultMoveTilesErrorRange, 1.5);
        turnToAngle(45.0, -halfRobotLengthIn * 0.3, defaultTurnAngleErrorRange, 1.0);
        setRightWingState(false);
        task::sleep(500);
        turnToAngle(-45, 0.0, defaultTurnAngleErrorRange, 1.5);
        
        // Prepare to push balls over to the offensive zone
        task::sleep(500);
        driveAndTurnDistanceTiles(-1.00, -90.0, 30.0, 8.0, defaultMoveTilesErrorRange, 1.7);

        // Idle until 11 seconds of autonomous
        while (autontimer.value() < 11.0) {
            task::sleep(20);
        }

        // Push the preload and former-corner ball over to the offensive zone
        driveAndTurnDistanceTiles(-1.50, -90.0, 40.0, 100.0, defaultMoveTilesErrorRange, 2.0);

        // Touch the elevation bar
        task::sleep(500);
        turnToAngle(-98.0, 0.0, defaultTurnAngleErrorRange, 1.5);
        // Robot is touching the elevation bar
    }

    /// @brief Run the 15-seconds near-side AWP-safe autonomous.
    void runAutonNearAWPSafe() {

    }

    /// @brief Run the 15-seconds far-side AWP autonomous.
    void runAutonFarAWP() {
        setRotation(0.0);

        // Go near the center
        driveAndTurnDistanceTiles(1.80, -5.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.7);
        // Face goal and release preload ball
        turnToAngle(45.0, halfRobotLengthIn, defaultTurnAngleErrorRange, 1.0);
        task::sleep(200);
        setIntakeState(0);

        // Face center-down ball
        driveAndTurnDistanceTiles(-0.96, 40.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        turnToAngle(-90.0, 0.0, defaultTurnAngleErrorRange, 1.5);
        // Intake ball
        driveAndTurnDistanceTiles(0.94, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.6);
        setIntakeState(1);
        task::sleep(200);
        // Face goal, drive, and release ball
        turnToAngle(60.0, 0.0, defaultTurnAngleErrorRange, 0.8);
        driveAndTurnDistanceTiles(0.90, 60.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        task::sleep(100);
        setIntakeState(0);
        task::sleep(200);
        driveAndTurnDistanceTiles(-0.60, 60.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);

        // Face center-middle ball
        turnToAngle(-25.0, 0.0, defaultTurnAngleErrorRange, 0.7);
        // Intake ball
        driveAndTurnDistanceTiles(0.64, -23.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        turnToAngle(-90.0, -halfRobotLengthIn * 1.10, defaultTurnAngleErrorRange, 1.0);
        driveAndTurnDistanceTiles(0.60, -90.0, 75.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        setIntakeState(1);
        task::sleep(100);
        // Push balls through left-side of the goal
        setBackWingsState(true);
        driveAndTurnDistanceTiles(-2.20, -90.0, 80.0, 300.0, defaultMoveTilesErrorRange, 2.0);
        setBackWingsState(false);
        // Release loaded ball into the goal
        turnToAngle(90.0, 0.0, defaultTurnAngleErrorRange, 0.7);
        setIntakeState(0);
        driveAndTurnDistanceTiles(-0.30, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        setIntakeState(1);
        driveAndTurnDistanceTiles(-0.90, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        
        // Intake ball below elevation bar (not included)

        // Go to match load zone
        turnToAngle(190.0, 0.0, defaultTurnAngleErrorRange, 0.7);
        driveAndTurnDistanceTiles(1.80, 150.0, 50.0, 90.0, defaultMoveTilesErrorRange, 1.7);
        // Swing the corner ball out
        setRightWingState(true);
        turnToAngle(10.0, -halfRobotLengthIn * 1.4, defaultTurnAngleErrorRange, 1.7);
        turnToAngle(-30.0, -halfRobotLengthIn * 0.6, defaultTurnAngleErrorRange, 1.0);
        setRightWingState(false);
        // Push ball through the bottom-side of the goal
        turnToAngle(-165.0, 0.0, defaultTurnAngleErrorRange, 1.0);
        driveAndTurnDistanceTiles(-1.30, -180.0, 100.0, 1000.0, defaultMoveTilesErrorRange, 1.3);
        driveAndTurnDistanceTiles(0.90, -180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
    }

    /// @brief Run the 15-seconds far-side Eliminations autonomous.
    void runAutonFarElim() {
        
    }

    /// @brief Run the skills autonomous.
    void runAutonSkills() {
        setRotation(135.0); // Right, near-side starting position
        timer autonTimer;

        // Position for match-load
        driveAndTurnDistanceTiles(-0.80, 135.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
        turnToAngle(75.0, 0.0, defaultTurnAngleErrorRange, 0.6);
        setRightWingState(true);

        // Match-load 46 balls until at most 27 seconds of skills passed
        while (getPunchedCount() < 46 && autonTimer.value() < 27 && autonTimer.value() < 5) {
            task::sleep(10);
        }

        // TODO: lower the time elapsed in each section
        
        // Push the right, near-side corner triball out (3.6 s)
        setRightWingState(false);
        driveAndTurnDistanceTiles(0.30, 45.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
        setLeftWingState(true, 0.4);
        turnToAngleVelocity(-45.0, 75.0, halfRobotLengthIn * 0.70, defaultTurnAngleErrorRange, 1.5);
        setLeftWingState(false, 0.6);
        driveAndTurnDistanceTiles(-0.50, -90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 0.6);
        turnToAngle(-70.0, 0.0, defaultTurnAngleErrorRange, 0.5);

        // Push the right, far-side corner triball out, and push balls into the goal from the right (7.4 s)
        driveAndTurnDistanceTiles(-1.90, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.6);
        setRightWingState(true);
        driveAndTurnDistanceTiles(-0.80, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        driveAndTurnDistanceTiles(-1.25, -135.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        setBackWingsState(true);
        driveAndTurnDistanceTiles(-0.50, -180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
        // Push balls into goal
        setLeftWingState(false);
        turnToAngle(-160.0, 0.0, defaultTurnAngleErrorRange, 0.5);
        driveAndTurnDistanceTiles(-1.20, -180.0, 90.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        driveAndTurnDistanceTiles(0.40, -180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
        setBackWingsState(false);
        turnToAngle(-170.0, 0.0, defaultTurnAngleErrorRange, 0.4);
        driveAndTurnDistanceTiles(-1.00, -180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);

        // Push balls into the goal from the right-bottom (7.0 s)
        turnToAngle(-75.0, halfRobotLengthIn * 0.50, defaultTurnAngleErrorRange, 1.0);
        driveAndTurnDistanceTiles(1.50, -75.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.3);
        setFrontWingsState(true, 1.0);
        turnToAngleVelocity(80.0, 75.0, halfRobotLengthIn * 0.90, defaultTurnAngleErrorRange, 1.5);
        driveAndTurnDistanceTiles(1.50, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-0.50, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
        turnToAngle(90.0, 0.0, defaultTurnAngleErrorRange, 0.5);
        setFrontWingsState(false, 0.4);
        driveAndTurnDistanceTiles(0.70, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        // Again, but from the left-bottom (7.1 s)
        turnToAngle(90.0, 0.0, defaultTurnAngleErrorRange, 0.5);
        driveAndTurnDistanceTiles(-0.75, 75.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
        turnToAngle(180.0, -halfRobotLengthIn * 1.20, defaultTurnAngleErrorRange, 0.7);
        setLeftWingState(true);
        driveAndTurnDistanceTiles(-0.70, 180.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
        turnToAngle(290.0, -halfRobotLengthIn * 0.60, defaultTurnAngleErrorRange, 0.7);
        setBackWingsState(true);
        driveAndTurnDistanceTiles(-2.00, 270.0, 75.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(0.50, 270.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
        turnToAngle(270.0, 0.0, defaultTurnAngleErrorRange, 0.5);
        driveAndTurnDistanceTiles(-1.00, 270.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);

        // Push the left, far-side corner triball out, and push balls into the goal from the left (7.8 s)
        setBackWingsState(false, 0.3);
        driveAndTurnDistanceTiles(0.30, 270.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
        turnToAngle(180.0, 0.0, defaultTurnAngleErrorRange, 1.0);
        driveAndTurnDistanceTiles(-1.60, 180.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        turnToAngleVelocity(315.0, 60.0, -halfRobotLengthIn * 0.70, defaultTurnAngleErrorRange, 1.5);
        setBackWingsState(true);
        driveAndTurnDistanceTiles(-0.50, 360.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
        // Push balls into goal
        setRightWingState(false);
        turnToAngle(340.0, 0.0, defaultTurnAngleErrorRange, 0.5);
        driveAndTurnDistanceTiles(-1.20, 360.0, 75.0, 100.0, defaultMoveTilesErrorRange, 0.7);
        driveAndTurnDistanceTiles(0.40, 360.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
        setBackWingsState(false);
        turnToAngle(350.0, 0.0, defaultTurnAngleErrorRange, 0.4);
        driveAndTurnDistanceTiles(-1.00, 360.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.7);

        // Elevation

        // Prepare for elevation (2.5 s)
        driveAndTurnDistanceTiles(0.30, 350.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
        setLiftState(true);
        turnToAngle(310.0, -halfRobotLengthIn * 0.5, defaultTurnAngleErrorRange, 1.5);

        // Drive to elevation bar
        driveAndTurnDistanceTiles(3.30, 270, 80.0, 100.0, defaultMoveTilesErrorRange, 2.8);
        setLiftState(false);
    }

    void runAutonSkillsCrossBarrier() {
        setRotation(45.0); // Left, near-side starting position
        timer autonTimer;

        // Position for match-load
        turnToAngle(105.0, -halfRobotLengthIn * 1.20, defaultTurnAngleErrorRange, 0.6);
        driveAndTurnDistanceTiles(-0.30, 105.0, 15.0, 100.0, defaultMoveTilesErrorRange, 0.4);

        // Match-load 46 balls until at most 27 seconds of skills passed
        while (getPunchedCount() < 46 && autonTimer.value() < 27) {
            task::sleep(10);
        }

        // Scoop middle triballs over the short barrier (4.7 s)
        driveAndTurnDistanceTiles(1.10, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.2);
        setFrontWingsState(true);       
        driveAndTurnDistanceTiles(0.50, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.2);
    }
}