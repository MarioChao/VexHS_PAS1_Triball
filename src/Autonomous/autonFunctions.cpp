#include "Autonomous/autonFunctions.h"
#include "Utilities/angleFunctions.h"
#include "Utilities/pidControl.h"
#include "Utilities/motionProfile.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"
#include "Mechanics/botIntake.h"
#include "main.h"

namespace {
    using namespace angle;
    using namespace botinfo;
    using namespace field;

    void driveVelocity(double leftVelocityPct, double rightVelocityPct);

    double setLeftWing_DelaySec;
    double setRightWing_DelaySec;
    double setWings_DelaySec;
    bool setLeftWing_LeftWingState;
    bool setRightWing_RightWingState;
    bool setWings_WingsState;
}

namespace auton {
    /// @brief Set the inertial sensor's absolute angle reading to a specified value. Doesn't turn the robot.
    /// @param rotation The angle (in degrees) to be set for the current orientation.
    void setRotation(double rotation) {
        InertialSensor.setRotation(rotation, deg);
    }

    /// @brief Turn the robot to face a specified angle.
    /// @param rotation The target angle to face in degrees.
    /// @param rotateCenterOffsetIn The offset of the center of rotation.
    /// @param errorRange The allowed degree errors the target angle.
    /// @param runTimeout Maximum seconds the function will run for.
    void turnToAngle(double rotation, double rotateCenterOffsetIn, double errorRange, double runTimeout) {
        turnToAngleVelocity(rotation, 100.0, rotateCenterOffsetIn, errorRange, runTimeout);
    }

    /// @brief Turn the robot to face a specified angle.
    /// @param rotation The target angle to face in degrees.
    /// @param maxVelocityPct Maximum velocity of the rotation.
    /// @param rotateCenterOffsetIn The offset of the center of rotation.
    /// @param errorRange The allowed degree errors the target angle.
    /// @param runTimeout Maximum seconds the function will run for.
    void turnToAngleVelocity(double rotation, double maxVelocityPct, double rotateCenterOffsetIn, double errorRange, double runTimeout) {
        // Center of rotations
        double leftRotateRadiusIn = halfRobotLengthIn + rotateCenterOffsetIn;
        double rightRotateRadiusIn = halfRobotLengthIn - rotateCenterOffsetIn;
        double averageRotateRadiusIn = (leftRotateRadiusIn + rightRotateRadiusIn) / 2;

        // Velocity factors
        double leftVelocityFactor = leftRotateRadiusIn / averageRotateRadiusIn;
        double rightVelocityFactor = -rightRotateRadiusIn / averageRotateRadiusIn;
        
        // PID
        // L_vel = L_dist / time
        // R_vel = R_dist / time = L_vel * (R_dist / L_dist)
        PIDControl rotateTargetAnglePid(0.487, 0, 0, errorRange); // Reach goal
        timer timeout;
        while (!rotateTargetAnglePid.isSettled() && timeout.value() < runTimeout) {
            // Compute rotate error
            double rotateError = rotation - InertialSensor.rotation();
            rotateTargetAnglePid.computeFromError(rotateError);

            // Compute motor rotate velocities
            double averageMotorVelocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, rotateTargetAnglePid.getValue()));
            double leftMotorVelocityPct = leftVelocityFactor * averageMotorVelocityPct;
            double rightMotorVelocityPct = rightVelocityFactor * averageMotorVelocityPct;

            // Drive with velocities
            driveVelocity(leftMotorVelocityPct, rightMotorVelocityPct);

            task::sleep(20);
        }

        // Stop
        LeftRightMotors.stop(brake);
    }

    /// @brief Drive straight in the direction of the robot for a specified tile distance.
    /// @param distanceTiles Distance in units of tiles.
    /// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
    /// @param errorRange The allowed tile errors from the target distance.
    /// @param runTimeout Maximum seconds the function will run for.
    void driveDistanceTiles(double distanceTiles, double maxVelocityPct, double errorRange, double runTimeout) {
        driveAndTurnDistanceTiles(distanceTiles, InertialSensor.rotation(), maxVelocityPct, 100.0, errorRange, runTimeout);
    }

    /// @brief Drive the robot for a specified tile distance and rotate it to a specified rotation in degrees.
    /// @param distanceTiles Distance in units of tiles.
    /// @param targetRotation The target angle to face in degrees.
    /// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
    /// @param maxTurnVelocityPct Maximum rotational velocity of the drive. (can > 100)
    /// @param errorRange The allowed tile errors from the target distance.
    /// @param runTimeout Maximum seconds the function will run for.
    void driveAndTurnDistanceTilesMotionProfile(double distanceTiles, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double errorRange, double runTimeout) {
        driveAndTurnDistanceWithInchesMotionProfile(distanceTiles * tileLengthIn, targetRotation, maxVelocityPct, maxTurnVelocityPct, errorRange * tileLengthIn, runTimeout);
    }

    /// @brief Drive the robot for a specified tile distance and rotate it to a specified rotation in degrees.
    /// @param distanceTiles Distance in units of tiles.
    /// @param targetRotation The target angle to face in degrees.
    /// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
    /// @param maxTurnVelocityPct Maximum rotational velocity of the drive. (can > 100)
    /// @param errorRange The allowed tile errors from the target distance.
    /// @param runTimeout Maximum seconds the function will run for.
    void driveAndTurnDistanceTiles(double distanceTiles, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double errorRange, double runTimeout) {
        driveAndTurnDistanceWithInches(distanceTiles * tileLengthIn, targetRotation, maxVelocityPct, maxTurnVelocityPct, errorRange * tileLengthIn, runTimeout);
    }

    /// @brief Drive the robot for a specified distance in inches and rotate it to a specified rotation in degrees.
    /// @param distanceInches Distance in units of inches.
    /// @param targetRotation The target angle to face in degrees.
    /// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
    /// @param maxTurnVelocityPct Maximum rotational velocity of the drive. (can > 100)
    /// @param errorRange The allowed inch errors from the target distance.
    /// @param runTimeout Maximum seconds the function will run for.
    void driveAndTurnDistanceWithInches(double distanceInches, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double errorRange, double runTimeout) {
        // Test
        // driveAndTurnDistanceInchesMotionProfile(distanceInches, targetRotation, maxVelocityPct, maxTurnVelocityPct, errorRange, runTimeout);
        // return;
        
        // Variables
        double motorTargetDistanceRev = distanceInches * (1.0 / driveWheelCircumIn) * (driveWheelMotorGearRatio);
        double leftMotorInitRev = LeftMotors.position(rev);
        double rightMotorInitRev = RightMotors.position(rev);

        // PID
        PIDControl driveTargetDistancePid(25, 0, 0, errorRange);
        PIDControl rotateTargetAnglePid(0.6, 0, 0, defaultTurnAngleErrorRange);

        timer timeout;
        while ((!driveTargetDistancePid.isSettled() || !rotateTargetAnglePid.isSettled()) && timeout.value() < runTimeout) {
            // Compute average traveled motor revolutions
            double leftTravelRev = LeftMotors.position(rev) - leftMotorInitRev;
            double rightTravelRev = RightMotors.position(rev) - rightMotorInitRev;
            double averageTravelRev = (leftTravelRev + rightTravelRev) / 2;

            // Compute motor velocity pid-value from error
            double distanceError = (motorTargetDistanceRev - averageTravelRev);
            driveTargetDistancePid.computeFromError(distanceError);
            double velocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, driveTargetDistancePid.getValue()));

            // Compute heading pid-value from error
            double rotateError = (targetRotation - InertialSensor.rotation());
            rotateTargetAnglePid.computeFromError(rotateError);
            double rotateVelocityPct = fmin(maxTurnVelocityPct, fmax(-maxTurnVelocityPct, rotateTargetAnglePid.getValue()));

            // Compute final motor velocities
            double leftVelocityPct = velocityPct + rotateVelocityPct;
            double rightVelocityPct = velocityPct - rotateVelocityPct;
            
            // Drive with velocities
            driveVelocity(leftVelocityPct, rightVelocityPct);

            task::sleep(20);
        }

        // Stop
        LeftRightMotors.stop(brake);
    }

    /// @brief Drive the robot for a specified distance in inches and rotate it to a specified rotation in degrees.
    /// @param distanceInches Distance in units of inches.
    /// @param targetRotation The target angle to face in degrees.
    /// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
    /// @param maxTurnVelocityPct Maximum rotational velocity of the drive. (can > 100)
    /// @param errorRange The allowed inch errors from the target distance.
    /// @param runTimeout Maximum seconds the function will run for.
    void driveAndTurnDistanceWithInchesMotionProfile(double distanceInches, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double errorRange, double runTimeout) {
        // Variables
        double motorTargetDistanceRev = distanceInches / driveWheelCircumIn * (driveWheelMotorGearRatio);
        double leftMotorInitRev = LeftMotors.position(rev);
        double rightMotorInitRev = RightMotors.position(rev);

        // PID
        PIDControl driveTargetDistancePid(10, 0, 0, errorRange);
        PIDControl rotateTargetAnglePid(0.6, 0, 0, defaultTurnAngleErrorRange);
        // Motion profile
        MotionProfile driveSpeedMotionProfile;
        driveSpeedMotionProfile.setModeAcceleration(90, 90, maxVelocityPct);
        driveSpeedMotionProfile.createProfile(motorTargetDistanceRev * driveMotorRevToPercentSecFactor);
        driveSpeedMotionProfile.start();
        printf(" --- Motion running for %.3f seconds. ---\n", driveSpeedMotionProfile.getMotionEndTime());

        timer timeout;
        while ((!driveSpeedMotionProfile.isDone() || !driveTargetDistancePid.isSettled() || !rotateTargetAnglePid.isSettled()) && timeout.value() < runTimeout) {
            // Compute average traveled motor revolutions
            double leftTravelRev = LeftMotors.position(rev) - leftMotorInitRev;
            double rightTravelRev = RightMotors.position(rev) - rightMotorInitRev;
            double averageTravelRev = (leftTravelRev + rightTravelRev) / 2;

            // Compute velocity pid-value from error
            double distanceError = driveSpeedMotionProfile.getDistance() / (driveMotorRevToPercentSecFactor) - averageTravelRev;
            driveTargetDistancePid.computeFromError(distanceError);

            // Compute motor velocity from motion profile & pid-value
            double linearVelocityPct = driveSpeedMotionProfile.getVelocity() + driveTargetDistancePid.getValue();
            linearVelocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, linearVelocityPct));

            // Compute heading pid-value from error
            double rotateError = (targetRotation - InertialSensor.rotation());
            rotateTargetAnglePid.computeFromError(rotateError);
            double rotateVelocityPct = fmin(maxTurnVelocityPct, fmax(-maxTurnVelocityPct, rotateTargetAnglePid.getValue()));

            // Compute final motor velocities
            double leftVelocityPct = linearVelocityPct + rotateVelocityPct;
            double rightVelocityPct = linearVelocityPct - rotateVelocityPct;

            // Drive with velocities
            driveVelocity(leftVelocityPct, rightVelocityPct);

            task::sleep(20);
        }

        // Stop
        LeftRightMotors.stop(brake);
    }

    /// @brief Set the state of the intake pneumatic.
    /// @param state Holding: 1, released: 0
    void setIntakeState(int state) {
        // Actual resolve state is inversed (holding : 0, released : 1)
        setIntakeResolveState(state ^ 1);
    }

    /// @brief Set the state of Left Wing's pneumatic.
    /// @param state Expanded: true, retracted: false.
    /// @param delaySec Number of seconds to wait before setting the pneumatic set (in a task).
    void setLeftWingState(bool state, double delaySec) {
        setLeftWing_LeftWingState = state;
        setLeftWing_DelaySec = delaySec;
        task setPneumaticState([] () -> int {
            if (setLeftWing_DelaySec > 1e-9) {
                task::sleep(setLeftWing_DelaySec * 1000);
            }
            LeftWingPneumatic.set(setLeftWing_LeftWingState);
            return 1;
        });
    }

    /// @brief Set the state of Right Wing's pneumatic.
    /// @param state Expanded: true, retracted: false.
    /// @param delaySec Number of seconds to wait before setting the pneumatic set (in a task).
    void setRightWingState(bool state, double delaySec) {
        setRightWing_RightWingState = state;
        setRightWing_DelaySec = delaySec;
        task setPneumaticState([] () -> int {
            if (setRightWing_DelaySec > 1e-9) {
                task::sleep(setRightWing_DelaySec * 1000);
            }
            RightWingPneumatic.set(setRightWing_RightWingState);
            return 1;
        });
    }

    /// @brief Set the state of Left and Right Wing's pneumatic.
    /// @param state Expanded: true, retracted: false.
    /// @param delaySec Number of seconds to wait before setting the pneumatic set (in a task).
    void setWingsState(bool state, double delaySec) {
        setWings_WingsState = state;
        setWings_DelaySec = delaySec;
        task setPneumaticsState([] () -> int {
            if (setWings_DelaySec > 1e-9) {
                task::sleep(setWings_DelaySec * 1000);
            }
            LeftWingPneumatic.set(setWings_WingsState);
            RightWingPneumatic.set(setWings_WingsState);
            return 1;
        });
    }

    /// @brief Set the state of the lift's pneumatic.
    /// @param state Lifted: true, lowered: false
    void setLiftState(bool state) {
        LiftPneumatic.set(state);
    }    
}

namespace {
    void driveVelocity(double leftVelocityPct, double rightVelocityPct) {
        // Scale percentages if overshoot
        double scaleFactor = 100.0 / fmax(100.0, fmax(fabs(leftVelocityPct), fabs(rightVelocityPct)));
        leftVelocityPct *= scaleFactor;
        rightVelocityPct *= scaleFactor;

        // Spin motors
        LeftMotors.spin(fwd, leftVelocityPct, pct);
        RightMotors.spin(fwd, rightVelocityPct, pct);
    }
}