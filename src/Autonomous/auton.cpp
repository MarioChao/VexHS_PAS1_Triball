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
    void runAutonNearAWPBackside();
    void runAutonNearElim();
    void runAutonFarAWP();
    void runAutonFarElim();
    void runAutonSkills();

    autonomousType auton_runType = autonomousType::None;
    int auton_allianceId;
}


void setAutonRunType(int allianceId, autonomousType autonType) {
    switch (autonType) {
        case autonomousType::NearAWP:
            printOnController("Auton: NearAWP");
            break;
        case autonomousType::NearElim:
            printOnController("Auton: NearElim");
            break;
        case autonomousType::FarAWP:
            printOnController("Auton: FarAWP");
            break;
        case autonomousType::FarElim:
            printOnController("Auton: FarElim");
            break;
        case autonomousType::Skills:
            printOnController("Auton: Skills");
            break;
        default:
            printOnController("Auton: None");
            break;
    }
    auton_runType = autonType;
    auton_allianceId = allianceId;
}

void runAutonomous() {
    switch (auton_runType) {
        case autonomousType::NearAWP:
            // runAutonNearAWP();
            runAutonNearAWPBackside();
            break;
        case autonomousType::NearElim:
            runAutonNearElim();
            break;
        case autonomousType::FarAWP:
            runAutonFarAWP();
            break;
        case autonomousType::FarElim:
            runAutonFarElim();
            break;
        case autonomousType::Skills:
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
        // driveAndTurnDistanceTiles(3.0, 90.0, 100.0, 100.0, 0.05, 3.0);
        turnToAngle(90);
        task::sleep(200);
        turnToAngle(-90);
        task::sleep(200);
        turnToAngle(180);
        task::sleep(200);
        turnToAngle(-180);
        task::sleep(200);
        turnToAngle(0);
        task::sleep(200);
        // driveDistanceTiles(3);
        // task::sleep(200);
        // driveDistanceTiles(-3);
        // task::sleep(200);
    }

    /// @brief Run the 15-seconds near-side AWP autonomous.
    void runAutonNearAWP() {
        // For PASVEX's Robot
        setRotation(0.0);
        
        // Intake middle ball
        driveAndTurnDistanceTiles(2.3, 15.0, 800.0, 100.0, 0.05, 1.5);
        setIntakeState(1);
        turnToAngle(30.0, halfRobotLengthIn, 5.0, 1.5);

        // Push middle ball over the middle barrier
        turnToAngle(90.0, -halfRobotLengthIn * 0.5, 5.0, 1.0);
        turnToAngle(270.0, 0.0, 5.0, 1.5);
        setIntakeState(0);
        setRightWingState(true);
        driveAndTurnDistanceTiles(-1.5, 270.0, 100.0, 100.0, 0.05, 1.3);
        setRightWingState(false);

        // Drive to match load zone
        driveAndTurnDistanceTiles(0.3, 270.0, 100.0, 100.0, 0.05, 1.5);
        turnToAngle(215.0, 0.0, 5.0, 1.5);
        driveAndTurnDistanceTiles(2.1, 215.0, 100.0, 100.0, 0.05, 2.0);
        // Face upward-parallel to the match load bar
        turnToAngle(315.0, halfRobotLengthIn, 5.0, 1.5);
        // Swing the ball out
        setLeftWingState(true);
        driveAndTurnDistanceTiles(-1.0, 270.0, 100.0, 400.0, 0.05, 1.5);
        setLeftWingState(false);
        
        // Push the preload and former-corner ball over to the offensive zone
        driveAndTurnDistanceTiles(-1.3, 270.0, 100.0, 100.0, 0.05, 1.5);

        // Touch the elevation bar
        setRightWingState(true);
        driveAndTurnDistanceTiles(-0.25, 270.0, 50.0, 100.0, 0.05, 1.5);
        turnToAngle(275.0, 0.0, 5.0, 1.5);
        // Robot is touching the elevation bar

    }

    void runAutonNearAWPBackside() {
        // For PASVEX's Robot
        setRotation(180.0);
        
        // Push middle balls over the middle barrier
        driveAndTurnDistanceTiles(-1.8, 195.0, 800.0, 100.0, 0.05, 1.5);
        setRightWingState(true);
        turnToAngle(270.0, -halfRobotLengthIn, 5.0, 1.0);
        setRightWingState(true);
        driveAndTurnDistanceTiles(-1.5, 270.0, 100.0, 100.0, 0.05, 1.3);
        setRightWingState(false);

        // Drive to match load zone
        driveAndTurnDistanceTiles(0.3, 270.0, 100.0, 100.0, 0.05, 1.5);
        turnToAngle(215.0, 0.0, 5.0, 1.5);
        driveAndTurnDistanceTiles(2.1, 215.0, 100.0, 100.0, 0.05, 2.0);
        // Face upward-parallel to the match load bar
        turnToAngle(315.0, halfRobotLengthIn, 5.0, 1.5);
        // Swing the ball out
        setLeftWingState(true);
        driveAndTurnDistanceTiles(-1.0, 270.0, 100.0, 400.0, 0.05, 1.5);
        setLeftWingState(false);
        
        // Push the preload and former-corner ball over to the offensive zone
        driveAndTurnDistanceTiles(-1.3, 270.0, 100.0, 100.0, 0.05, 1.5);

        // Touch the elevation bar
        setRightWingState(true);
        driveAndTurnDistanceTiles(-0.25, 270.0, 50.0, 100.0, 0.05, 1.5);
        turnToAngle(275.0, 0.0, 5.0, 1.5);
        // Robot is touching the elevation bar

    }

    /// @brief Run the 15-seconds near-side Eliminations autonomous.
    void runAutonNearElim() {
        setRotation(36);

        // Expand left wing to push preload triball
        setLeftWingState(true);
        task retractWing([] () -> int {
            task::sleep(150);
            setLeftWingState(false);
            return 1;
        });

        // Push the middle balls over the barrier
        driveAndTurnDistanceTiles(2.42, -10.0, 100.0, 560.0, 0.05, 2.3);
        setLeftWingState(true);
        turnToAngle(91.5, halfRobotLengthIn * 0.5, 3.0, 1.0); // Face the barrier
        driveAndTurnDistanceTiles(1.72, 90.0, 100.0, 100.0, 0.05, 1.4);
        setLeftWingState(false);

        // Push the corner triball out
        driveDistanceTiles(-0.5, 100.0, 0.05, 1.0);
        turnToAngle(33, halfRobotLengthIn, 3.0, 1.0); // Face the matchload zone
        driveAndTurnDistanceTiles(-sqrt(pow(1.9, 2) + pow(1.9, 2)), 33, 100.0, 70.0, 0.05, 2.5);
        turnToAngle(135, -halfRobotLengthIn * 0.7, 3.0, 1.0); // Face parallel to the matchload zone
        // setAnchorState(true);
        task::sleep(100);
        turnToAngle(75, -halfRobotLengthIn * 1.5, 3.0, 1.0); // Anchor push the triball
        // setAnchorState(false);

        // Push the former-corner and elevation-bar balls
        turnToAngle(-75, 0.0, 3.0, 1.0); // Back-side face down-right (more right)
        driveAndTurnDistanceTiles(-1.75, -90, 100.0, 200.0, 0.05, 2.5);
        
        // Go to matchload position
        driveAndTurnDistanceTiles(2.5, -45, 100.0, 240.0, 0.05, 2.5);
        turnToAngle(110, 0.0, 3.0, 1.0); // Face down-right (more right)
        // setAnchorState(true);
        task::sleep(100);
        turnToAngle(90, halfRobotLengthIn * 3.0, 3.0, 1.0);
    }

    /// @brief Run the 15-seconds far-side AWP autonomous.
    void runAutonFarAWP() {
        setRotation(-90.0);
        
        // Intake ball below elevation bar
        setIntakeState(1);
        driveAndTurnDistanceTiles(0.4, -90.0, 100.0, 100.0, 0.05, 1.5);

        // Go to match load zone while pushing preload ball
        driveAndTurnDistanceTiles(-2.2, -135.0, 80.0, 8.0, 0.05, 2.0);
        // Swing the corner ball out
        setLeftWingState(true);
        turnToAngle(-180.0, halfRobotLengthIn, 5.0, 1.0);
        // Turn around
        setLeftWingState(false);
        turnToAngle(-335.0, 0.0, 5.0, 1.5);
        setIntakeState(0);
        // Push three balls through the bottom-side of the goal
        driveAndTurnDistanceTiles(1.5, -360.0, 100.0, 300.0, 0.05, 1.0);

        // Face center-down ball
        driveAndTurnDistanceTiles(-1.0, -270.0, 100.0, 100.0, 0.05, 1.5);
        turnToAngle(-70.0, 0.0, 5.0, 1.5);
        // Intake ball
        setIntakeState(1);
        driveAndTurnDistanceTiles(1.7, -80.0, 100.0, 200.0, 0.05, 1.5);
        // Face goal
        turnToAngle(75.0, 0.0, 5.0, 1.5);
        // Out-take ball
        setIntakeState(-1);
        task::sleep(200);
        setIntakeState(0);

        // Face center-middle ball
        turnToAngle(30.0, 0.0, 5.0, 1.5);
        // Intake ball
        setIntakeState(1);
        driveAndTurnDistanceTiles(1.3, -60.0, 100.0, 300.0, 0.05, 1.5);
        // Push two balls through left-side of the goal
        turnToAngle(-90.0, 0.0, 5.0, 1.5);
        setWingsState(true);
        driveAndTurnDistanceTiles(-2.0, -90.0, 100.0, 100.0, 0.05, 1.3);
        // Push loaded ball into the goal
        setWingsState(false);
        driveAndTurnDistanceTiles(0.5, -90.0, 100.0, 100.0, 0.05, 1.5);
        turnToAngle(90.0, 0.0, 5.0, 1.5);
        setIntakeState(0);
        driveAndTurnDistanceTiles(1.5, 90.0, 100.0, 100.0, 0.05, 1.0);
        
        // Face the elevation bar
        driveAndTurnDistanceTiles(-1.5, 20.0, 100.0, 100.0, 0.05, 2.0);
        // Drive and touch the bar using wings
        setLeftWingState(true);
        driveAndTurnDistanceTiles(-2.0, 0.0, 150.0, 100.0, 0.05, 2.0);
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
        setRotation(132);

        // Push the two alliance triballs
        driveAndTurnDistanceTiles(-2.2, 180, 100.0, 1200.0, 0.05, 1.5);

        // Go to matchload position
        task::sleep(300);
        driveAndTurnDistanceTiles(1.2, 140.0, 60.0, 900.0, 0.05, 1.5);
        // Prepare for matchload
        setFlywheelSpeedRpm(600);
        // Match load positioning
        turnToAngle(60, 0.0, 5.0, 1.5);
        driveAndTurnDistanceTiles(-0.5, 60, 100.0, 800.0, 0.05, 1.0);

        // Match load 44 balls
        task::sleep(2000);
        timer duration;
        // turnToAngleVelocity(65, 1.0, halfRobotLengthIn * 2.0, 3.0, 25.0);
        while (duration.value() < 25.0) {
            task::sleep(10);
        }
        // turnToAngleVelocity(45, 10.0, halfRobotLengthIn * 2.0, 3.0, 5.0);
        // while (duration.value() < 5.0) {
        //     task::sleep(10);
        // }
        // turnToAngleVelocity(45, 1, halfRobotLengthIn * 2.0, 3.0, 60.0);
        // while (duration.value() < 60.0) {
        //     task::sleep(10);
        // }

        // Finish matchload
        setFlywheelSpeedRpm(0);

        // Push the triballs through path that goes below the red elevation-bar
        turnToAngle(-60, 0.0, 3.0, 1.0);
        driveAndTurnDistanceTiles(-4.0, -90.0, 100.0, 300.0, 0.05, 3.3);

        // Score triballs through bottom-side of the goal
        turnToAngle(-115, 0, 3.0, 1.0);
        driveAndTurnDistanceTiles(-3.0, -180.0, 100.0, 400.0, 0.05, 2.2);
        driveAndTurnDistanceTiles(1.0, -155, 100.0, 300.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(-2.0, -180.0, 100.0, 700.0, 0.05, 1.7);
        
        // Score triballs through left-side of the goal
        driveAndTurnDistanceTiles(0.3, -180.0, 100.0, 100.0, 0.05, 1.5);
        turnToAngle(-70, halfRobotLengthIn * 0.5, 3.0, 1.5);
        driveAndTurnDistanceTiles(2.0, -90, 100.0, 300.0, 0.05, 1.5);
        turnToAngle(-135, 0, 3.0, 1.5);
        setWingsState(true);
        driveAndTurnDistanceTiles(-2.5, -90.0, 100.0, 300.0, 0.05, 1.5);
        setWingsState(false);

        // Score more triballs through left-side of the goal
        driveAndTurnDistanceTiles(1.5, -180, 800.0, 50.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(-2.4, -180, 100.0, 100.0, 0.05, 1.5);
        turnToAngle(-35, 0, 3.0, 1.5);
        setWingsState(true);
        driveAndTurnDistanceTiles(-2.5, -90, 100.0, 150.0, 0.05, 1.5);
        setWingsState(false);

        // Score triballs through top-side of the goal
        driveAndTurnDistanceTiles(0.5, 5.0, 100.0, 70.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(1.5, 20.0, 100.0, 150.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(1.0, -90.0, 100.0, 200.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(-1.5, -45.0, 100.0, 400.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(-2.0, 0.0, 100.0, 500.0, 0.05, 1.5);

        // Elevation
        driveAndTurnDistanceTiles(0.5, 0.0, 100.0, 100.0, 0.05, 1.5);
        // Prepare for elevation
        setLiftState(true);
        // Drive to elevation bar
        turnToAngle(-45, 0.0, 3.0, 1.5);
        driveAndTurnDistanceTiles(3.6, -90, 100.0, 300.0, 0.05, 3.0);
        // Clamp on the bar
        task::sleep(100);
        setLiftState(false);
    }
}