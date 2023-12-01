#include "Autonomous/auton.h"
#include "Autonomous/autonFunctions.h"
#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"
#include "main.h"

namespace {
    using namespace auton;
    using namespace botinfo;
    using debug::printOnController;

    void runAutonNearAWP();
    void runAutonNearElim();
    void runAutonFarAWP();
    void runAutonFarElim();
    void runAutonFarElim6Balls();
    void runAutonSkills();

    autonomousType auton_runType = autonomousType::NearElim;
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
            runAutonNearAWP();
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
        default:
            break;
    }
}

namespace {
    /// @brief Run the 15-seconds near-side AWP autonomous.
    void runAutonNearAWP() {
        setRotation(0);
        
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
        driveAndTurnDistanceTiles(2.42, -10.0, 5.6, 100.0, 0.05, 2.3);
        setLeftWingState(true);
        turnToAngle(91.5, halfRobotLengthIn * 0.5, 3.0, 1.0); // Face the barrier
        driveAndTurnDistanceTiles(1.72, 90.0, 1.0, 100.0, 0.05, 1.4);
        setLeftWingState(false);

        // Push the corner triball out
        driveDistanceTiles(-0.5, 100.0, 0.05, 1.0);
        turnToAngle(33, halfRobotLengthIn, 3.0, 1.0); // Face the matchload zone
        driveAndTurnDistanceTiles(-sqrt(pow(1.9, 2) + pow(1.9, 2)), 33, 0.7, 100.0, 0.05, 2.5);
        turnToAngle(135, -halfRobotLengthIn * 0.7, 3.0, 1.0); // Face parallel to the matchload zone
        setAnchorState(true);
        task::sleep(100);
        turnToAngle(75, -halfRobotLengthIn * 1.5, 3.0, 1.0); // Anchor push the triball
        setAnchorState(false);

        // Push the former-corner and elevation-bar balls
        turnToAngle(-75, 0.0, 3.0, 1.0); // Back-side face down-right (more right)
        driveAndTurnDistanceTiles(-1.75, -90, 2.0, 100.0, 0.05, 2.5);
        
        // Go to matchload position
        driveAndTurnDistanceTiles(2.5, -45, 2.4, 100.0, 0.05, 2.5);
        turnToAngle(110, 0.0, 3.0, 1.0); // Face down-right (more right)
        setAnchorState(true);
        task::sleep(100);
        turnToAngle(90, halfRobotLengthIn * 3.0, 3.0, 1.0);
    }

    /// @brief Run the 15-seconds far-side AWP autonomous.
    void runAutonFarAWP() {
        setRotation(-36);
        
        // Expand right wing to push preload triball
        setRightWingState(true);
        task retractWing([] () -> int {
            task::sleep(150);
            setRightWingState(false);
            return 1;
        });

        // Score the middle balls
        driveAndTurnDistanceTiles(sqrt(pow(2.35, 2) + pow(2.35, 2)), -36, 10.0, 100.0, 0.05, 2.3);
        // Intake middle ball
        setIntakeState(true);
        turnToAngle(-65, -halfRobotLengthIn, 3.0, 0.7); // Face up-left (more left)
        // Score the balls
        turnToAngle(90, 0, 3.0, 1.0); // Face the goal
        setIntakeState(false);
        setWingsState(true);
        driveAndTurnDistanceTiles(2.5, 90, 2.0, 100.0, 0.05, 1.5);
        setWingsState(false);

        // Go to matchload zone
        driveDistanceTiles(-0.5, 100.0, 0.05, 0.5);
        driveAndTurnDistanceTiles(-4.0, 0, 3.0, 100.0, 0.05, 2.0);
        turnToAngle(90, 3.0, 1.0);
        driveAndTurnDistanceTiles(0.7, 45, 1.0, 100.0, 0.05, 1.0);
        
        // Push the corner triball out
        setAnchorState(true);
        turnToAngle(0, -halfRobotLengthIn, 3.0, 1.0);
        setAnchorState(false);

        // Push the preload and former-corner triballs in the goal
        turnToAngle(10);
        setRightWingState(true);
        driveAndTurnDistanceTiles(1.5, 0, 0.7, 100.0, 0.05, 0.5);
        setRightWingState(false);

        // Touch the bar
        driveAndTurnDistanceTiles(-3.7, 90, 2.5, 100.0, 0.05, 2.0);
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
        driveAndTurnDistanceTiles(sqrt(pow(2.35, 2) + pow(2.35, 2)), -36, 10.0, 100.0, 0.05, 2.3);
        // Intake middle ball
        setIntakeState(true);
        turnToAngle(-65, -halfRobotLengthIn, 3.0, 0.7); // Face up-left (more left)
        // Score the balls
        turnToAngle(90, 0, 3.0, 1.0); // Face the goal
        setIntakeState(false);
        setWingsState(true);
        driveAndTurnDistanceTiles(2.5, 90, 2.0, 100.0, 0.05, 1.5);
        setWingsState(false);

        // Intake a triball from middle
        driveDistanceTiles(-0.5, 100.0, 0.05, 0.5);
        turnToAngle(234, 0, 3.0, 1.0); // Face the triball
        // Lift to prepare out-take
        setLiftPositionValue(0);
        setLiftToDegreeRotation(300, 2.0, false);
        liftToDegreeTask();
        // Intake ball
        setIntakeState(true);
        driveAndTurnDistanceTiles(sqrt(pow(1.32, 2) + pow(1.32, 2)), 234, 1.0, 100.0, 0.05, 1.3);
        turnToAngle(260, halfRobotLengthIn, 3.0, 0.7); // Face the down-left (more left)

        // Go to match load zone
        turnToAngle(310, -halfRobotLengthIn * 1.5, 3.0, 1.0); // Face the matchload zone
        // Move toward match load zone
        setIntakeState(true);
        driveAndTurnDistanceTiles(-sqrt(pow(1.65, 2) + pow(1.65, 2)), 310, 1.0, 100.0, 0.05, 2.0);

        // Push the corner triball out
        turnToAngle(405, -halfRobotLengthIn * 0.3, 4.0, 1.0); // Face parallel to the matchload zone
        setIntakeState(false);
        // Lower lift to outtake
        setLiftToDegreeRotation(-15, 3.0);
        liftToDegreeTask();
        // Anchor swing the ball
        setAnchorState(true);
        task::sleep(100);
        turnToAngle(345, -halfRobotLengthIn, 3.0, 1.0); // Face up-left (swing the ball out)
        setAnchorState(false);
        turnToAngle(375, 0.0, 1.0, 0.5); // Unstuck the anchor

        // Push the preload, former-corner, and released triballs into the goal
        // Push with back
        turnToAngle(210, 0, 3.0, 1.0);
        driveAndTurnDistanceTiles(-2.5, 180, 2.0, 100.0, 0.05, 0.7);
        driveAndTurnDistanceTiles(1.0, 205, 1.9, 100.0, 0.05, 1.0);
        driveAndTurnDistanceTiles(-2.0, 180, 2.0, 100.0, 0.05, 0.7);

        // Drive a little back
        driveDistanceTiles(0.3, 100.0, 0.05, 1.0);
    }

    /// @brief Run the skills autonomous.
    void runAutonSkills() {
        setRotation(132);

        // Push the two alliance triballs
        driveAndTurnDistanceTiles(-2.2, 180, 7.5, 100.0, 0.05, 1.5);

        // Go to matchload position
        driveAndTurnDistanceTiles(1.2, 140.0, 7.5, 100.0, 0.05, 1.0);
        // Prepare for matchload
        setLiftPositionValue(0);
        setLiftToDegreeRotation(125);
        liftToDegreeTask();
        setFlywheelSpeedRpm(600);
        // Matchload positioning
        turnToAngle(125, halfRobotLengthIn, 3.0, 1.0);
        setAnchorState(true);
        turnToAngle(85, halfRobotLengthIn * 1.3, 3.0, 1.0);
        driveDistanceTiles(-0.3, 60.0, 0.05, 0.7);
        setAnchorState(false);

        // Matchload 44 balls
        task::sleep(2000);
        timer duration;
        turnToAngleVelocity(65, 1.0, halfRobotLengthIn * 2.0, 3.0, 25.0);
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
        setLiftToDegreeRotation(0);
        liftToDegreeTask();
        setFlywheelSpeedRpm(0);

        // Push the triballs through path that goes below the red elevation-bar
        turnToAngle(-48, 0.0, 3.0, 1.0);
        driveAndTurnDistanceTiles(-5.5, -90.0, 3.0, 100.0, 0.05, 3.3);

        // Score triballs through bottom-side of the goal
        turnToAngle(-115, 0, 3.0, 1.0);
        driveAndTurnDistanceTiles(-3.0, -180.0, 1.0, 100.0, 0.05, 2.2);
        driveAndTurnDistanceTiles(1.0, -135, 5.0, 100.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(-2.0, -180.0, 7.0, 100.0, 0.05, 1.7);
        
        // Score triballs through left-side of the goal
        driveAndTurnDistanceTiles(0.5, -180.0, 1.0, 100.0, 0.05, 1.0);
        turnToAngle(-80, halfRobotLengthIn * 0.5, 3.0, 1.5);
        driveAndTurnDistanceTiles(2.2, -90, 3.0, 100.0, 0.05, 2.0);
        turnToAngle(45, halfRobotLengthIn, 3.0, 1.5);
        setWingsState(true);
        driveAndTurnDistanceTiles(2.5, 90.0, 4.0, 100.0, 0.05, 2.0);
        setWingsState(false);

        // Score more triballs through left-side of the goal
        turnToAngle(0, halfRobotLengthIn * 2.4, 3.0, 1.5);
        driveAndTurnDistanceTiles(2.3, 0, 1.0, 100.0, 0.05, 2.0);
        turnToAngle(135, 0, 3.0, 1.5);
        setWingsState(true);
        driveAndTurnDistanceTiles(2.5, 90, 2.0, 100.0, 0.05, 2.0);
        setWingsState(false);

        // Score triballs through top-side of the goal
        driveAndTurnDistanceTiles(-0.5, 5, 0.7, 100.0, 0.05, 2.0);
        driveAndTurnDistanceTiles(2.0, 20, 1.5, 100.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(1.0, -90, 1.5, 100.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(-1.5, -45, 2.0, 100.0, 0.05, 2.0);
        driveAndTurnDistanceTiles(-2.0, -90, 3.0, 100.0, 0.05, 2.0);

        // Elevation
        driveDistanceTiles(0.5, 100.0, 0.05, 0.5);
        // Prepare for elevation
        setLiftToDegreeRotation(450);
        liftToDegreeTask();
        // Drive to elevation bar
        turnToAngle(-45, 0.0, 3.0, 1.5);
        driveAndTurnDistanceTiles(3.6, -90, 3.0, 100.0, 0.05, 3.0);
        // Clamp on the bar
        task::sleep(100);
        // setLiftClampState(true);
        setLiftToDegreeRotation(-100, 30.0);
        liftToDegreeTask();
    }
}