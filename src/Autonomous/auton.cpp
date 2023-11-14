#include "Autonomous/auton.h"
#include "Autonomous/autonFunctions.h"
#include "Utilities/robotInfo.h"
#include "main.h"

namespace {
    using namespace auton;
    using namespace botinfo;

    void runAutonNearAWP();
    void runAutonNearElim();
    void runAutonFarAWP();
    void runAutonFarElim();
    void runAutonFarElim6Balls();
    void runAutonSkills();

    autonomousType auton_runType = autonomousType::Skills;
    int auton_allianceId;
}


void setAutonRunType(int allianceId, autonomousType autonType) {
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
        driveAndTurnDistanceTiles(2.64, -10.0, 5.6, 100.0, 0.05, 2.0);
        setLeftWingState(true);
        turnToAngle(91.5, 0.0, 3.0, 1.0); // Face the barrier
        driveAndTurnDistanceTiles(1.7, 91.5, 1.0, 100.0, 0.05, 0.7);
        setLeftWingState(false);

        // Push the corner triball out
        driveDistanceTiles(-0.5, 100.0, 0.05, 1.0);
        turnToAngle(33, halfRobotLengthIn, 3.0, 1.0); // Face the matchload zone
        driveAndTurnDistanceTiles(-sqrt(pow(1.75, 2) + pow(1.75, 2)), 33, 0.7, 100.0, 0.05, 2.0);
        turnToAngle(135, -halfRobotLengthIn * 0.7, 3.0, 1.0); // Face parallel to the matchload zone
        setAnchorState(true);
        task::sleep(100);
        turnToAngle(55, -halfRobotLengthIn * 1.5, 3.0, 1.0); // Anchor push the triball
        setAnchorState(false);

        // Push the former-corner and elevation-bar balls
        turnToAngle(-75, 0.0, 3.0, 1.0); // Back-side face down-right (more right)
        driveAndTurnDistanceTiles(-1.6, -90, 1.9, 100.0, 0.05, 2.5);
        
        // Go to matchload position
        driveAndTurnDistanceTiles(2.4, -45, 1.7, 100.0, 0.05, 2.5);
        turnToAngle(125, 0.0, 3.0, 1.0); // Face down-right (more right)
        setAnchorState(true);
        task::sleep(100);
        turnToAngle(90, halfRobotLengthIn * 2.8, 3.0, 1.0);
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
        driveDistanceTiles(sqrt(pow(2.33, 2) + pow(2.33, 2)), 100.0, 0.05, 1.7);
        setIntakeState(true);
        turnToAngle(-67, -halfRobotLengthIn, 3.0, 1.0); // Face up-left (more left)
        setIntakeState(false);
        turnToAngle(90, 0, 3.0, 1.0); // Face the goal
        setWingsState(true);
        driveAndTurnDistanceTiles(2.5, 90, 1.0, 100.0, 0.05, 0.7);
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
        driveAndTurnDistanceTiles(sqrt(pow(2.33, 2) + pow(2.33, 2)), -36, 1.0, 100.0, 0.05, 2.3);
        // Intake middle ball
        setIntakeState(true);
        turnToAngle(-67, -halfRobotLengthIn, 3.0, 1.0); // Face up-left (more left)
        setIntakeState(false);
        // Score the balls
        turnToAngle(90, 0, 3.0, 1.0); // Face the goal
        setWingsState(true);
        driveAndTurnDistanceTiles(2.5, 90, 1.0, 100.0, 0.05, 1.0);
        setWingsState(false);
        // Lift to prepare out-take
        setLiftPositionValue(0);
        setLiftToDegreeRotation(300, 2.0, false);
        liftToDegreeTask();

        // Intake a triball from middle
        driveDistanceTiles(-0.5, 100.0, 0.05, 0.5);
        turnToAngle(234, 0, 3.0, 1.0); // Face the triball
        setIntakeState(true);
        driveAndTurnDistanceTiles(sqrt(pow(1.32, 2) + pow(1.32, 2)), 234, 1.0, 100.0, 0.05, 1.5);
        turnToAngle(260, halfRobotLengthIn, 3.0, 1.0); // Face the down-left (more left)

        // Go to match load zone
        setIntakeState(false);
        turnToAngle(310, -halfRobotLengthIn * 1.5, 3.0, 1.0); // Face the matchload zone
        // Move toward match load zone
        driveAndTurnDistanceTiles(-sqrt(pow(1.65, 2) + pow(1.65, 2)), 310, 1.0, 100.0, 0.05, 2.0);

        // Push the corner triball out
        turnToAngle(395, 0, 3.0, 1.0); // Face under-parallel to the matchload zone
        // Lower lift to outtake
        setLiftToDegreeRotation(-15, 3.0);
        liftToDegreeTask();
        // Anchor swing the ball
        setAnchorState(true);
        turnToAngle(315, -halfRobotLengthIn, 3.0, 1.0); // Face up-left (swing the ball out)
        setAnchorState(false);

        // Push the preload, former-corner, and released triballs into the goal
        // Push with back
        turnToAngle(190, 0.0, 3.0, 1.0);
        driveAndTurnDistanceTiles(-2.5, 180, 0.7, 100.0, 0.05, 0.7);
        driveAndTurnDistanceTiles(1.2, 190, 1.0, 100.0, 0.05, 1.0);
        driveAndTurnDistanceTiles(-2.0, 180, 0.9, 100.0, 0.05, 0.7);

        // Drive a little back
        driveDistanceTiles(0.3, 100.0, 0.05, 1.0);
    }

    /// @brief Run the 15-seconds far-side Elimination 6 balls autonomous.
    void runAutonFarElim6Balls() {
        setRotation(-36);

        // Expand right wing to push preload triball
        setRightWingState(true);
        task retractWing([] () -> int {
            task::sleep(150);
            setRightWingState(false);
            return 1;
        });

        // Score the middle balls
        driveDistanceTiles(sqrt(pow(2.32, 2) + pow(2.32, 2)), 100.0, 0.05, 1.7);
        // Prepare out-take while intaking
        setLiftPositionValue(0);
        setLiftToDegreeRotation(300);
        liftToDegreeTask();
        // Intake middle ball
        setIntakeState(true);
        turnToAngle(-67, -halfRobotLengthIn, 3.0, 1.0); // Face up-left (more left)
        setIntakeState(false);
        // Score the balls
        turnToAngle(90, 0, 3.0, 1.0); // Face the goal
        setWingsState(true);
        driveAndTurnDistanceTiles(2.5, 90, 1.0, 100.0, 0.05, 1.0);
        setWingsState(false);

        // Intake a triball from middle
        driveDistanceTiles(-0.3, 100.0, 0.05, 0.5);
        setIntakeState(true);
        turnToAngle(232, 0, 3.0, 1.0); // Face the triball
        driveAndTurnDistanceTiles(sqrt(pow(1.32, 2) + pow(1.32, 2)), 232, 1.0, 100.0, 0.05, 1.0);
        turnToAngle(260, halfRobotLengthIn * 1.5, 3.0, 1.0); // Face the down-left (more left)
        setIntakeState(false);

        // Go to match load zone & release ball
        driveDistanceTiles(-0.3, 100.0, 0.05, 0.5);
        turnToAngle(125, 0, 3.0, 1.0); // Face the matchload zone
        // Outtake
        setLiftToDegreeRotation(-15, 1.0);
        liftToDegreeTask();
        // Move toward match load zone
        driveAndTurnDistanceTiles(sqrt(pow(1.4, 2) + pow(1.4, 2)), 125, 1.0, 100.0, 0.05, 1.5);

        // Intake elevation-bar ball
        turnToAngle(260, 0.0, 3.0, 1.0);
        setIntakeState(true);
        driveAndTurnDistanceTiles(1.3, 270, 2.0, 100.0, 0.05, 1.5);

        // Go to match load zone
        driveAndTurnDistanceTiles(-1.5, 260, 0.7, 100.0, 0.05, 1.5);

        // Push the corner triball out
        turnToAngle(395, 0, 3.0, 1.0); // Face under-parallel to the matchload zone
        // turnToAngle(405, 0, 3.0, 1.0); // Face parallel to the matchload zone
        setAnchorState(true);
        turnToAngle(315, -halfRobotLengthIn, 3.0, 1.0); // Face up-left (swing the ball out)
        setAnchorState(false);
        turnToAngle(370, -halfRobotLengthIn, 3.0, 1.0); // Face up-right (more right)

        // Push the preload, former-corner, released, and loaded triballs into the goal
        // setRightWingState(true);
        // task::sleep(100);
        driveAndTurnDistanceTiles(2.5, 360, 0.7, 100.0, 0.05, 0.7);
        // setRightWingState(false);

        // Push with back
        driveAndTurnDistanceTiles(-1.0, 360, 1.0, 100.0, 0.05, 0.5);
        turnToAngle(185, 0.0, 3.0, 1.0);
        driveAndTurnDistanceTiles(-2.0, 180, 1.0, 100.0, 0.05, 0.7);
        driveAndTurnDistanceTiles(1.0, 185, 1.0, 100.0, 0.05, 0.5);
        driveAndTurnDistanceTiles(-2.0, 180, 1.0, 100.0, 0.05, 0.7);

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
        turnToAngleVelocity(45, 2.0, halfRobotLengthIn * 2.0, 3.0, 25.0);
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
        driveAndTurnDistanceTiles(-5.7, -90.0, 3.0, 100.0, 0.05, 3.3);

        // Score triballs through bottom-side of the goal
        driveAndTurnDistanceTiles(-3.0, -180.0, 1.3, 100.0, 0.05, 1.7);
        driveAndTurnDistanceTiles(1.0, -190.0, 1.0, 100.0, 0.05, 1.0);
        driveAndTurnDistanceTiles(-2.0, -180.0, 1.3, 100.0, 0.05, 1.7);
        
        // Score triballs through left-side of the goal
        driveAndTurnDistanceTiles(2.0, -90, 2.5, 100.0, 0.05, 2.0);
        turnToAngle(45, halfRobotLengthIn, 3.0, 1.0);
        setWingsState(true);
        driveAndTurnDistanceTiles(2.0, 90.0, 5.0, 100.0, 0.05, 2.0);
        setWingsState(false);

        // Score more triballs through left-side of the goal
        turnToAngle(0, halfRobotLengthIn * 2.5, 3.0, 1.5);
        driveAndTurnDistanceTiles(2.3, 0, 1.0, 100.0, 0.05, 1.0);
        turnToAngle(135, 0.0, 3.0, 1.0);
        setWingsState(true);
        driveAndTurnDistanceTiles(2.0, 90, 5.0, 100.0, 0.05, 2.0);
        setWingsState(false);

        // Score triballs through top-side of the goal
        driveAndTurnDistanceTiles(-0.5, 0, 0.4, 100.0, 0.05, 1.5);
        driveAndTurnDistanceTiles(3.0, -90, 0.5, 100.0, 0.05, 2.5);
        driveAndTurnDistanceTiles(-2.5, 0, 2.0, 100.0, 0.05, 1.5);

        // Elevation
        driveDistanceTiles(0.5, 100.0, 0.05, 0.5);
        // Prepare for elevation
        setLiftToDegreeRotation(450);
        liftToDegreeTask();
        // Drive to elevation bar
        turnToAngle(-45, 0.0, 3.0, 1.5);
        driveAndTurnDistanceTiles(3.55, -90, 7.0, 100.0, 0.05, 3.0);
        // Clamp on the bar
        task::sleep(100);
        // setLiftClampState(true);
        setLiftToDegreeRotation(-100, 30.0);
        liftToDegreeTask();
    }
}