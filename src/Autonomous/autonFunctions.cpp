#include "Autonomous/autonFunctions.h"
#include "Utilities/angleFunctions.h"
#include "Utilities/pidControl.h"
#include "Utilities/motionProfile.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botFlywheel.h"
#include "main.h"

namespace {
    using namespace angle;
    using namespace botinfo;
    using namespace field;
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
        PIDControl rotateTargetAnglePid(0.5, 0, 0, errorRange); // Reach goal
        timer timeout;
        while (!rotateTargetAnglePid.isSettled() && timeout.value() < runTimeout) {
            // Compute rotate error
            double rotateError = rotation - InertialSensor.rotation();
            rotateTargetAnglePid.computeFromError(rotateError);

            // Compute motor rotate velocities
            double averageMotorVelocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, rotateTargetAnglePid.getValue()));
            double leftMotorVelocityPct = leftVelocityFactor * averageMotorVelocityPct;
            double rightMotorVelocityPct = rightVelocityFactor * averageMotorVelocityPct;

            // Scale percentages if overshoot
            double scaleFactor = 100.0 / fmax(100.0, fmax(fabs(leftMotorVelocityPct), fabs(rightMotorVelocityPct)));
            leftMotorVelocityPct *= scaleFactor;
            rightMotorVelocityPct *= scaleFactor;

            // Spin motors
            LeftMotors.spin(fwd, leftMotorVelocityPct, pct);
            RightMotors.spin(fwd, rightMotorVelocityPct, pct);

            // printf("Turning.. Now: %.3f, Err: %.3f\n", InertialSensor.rotation(deg), rotateError);
            task::sleep(20);
        }
        printf("Stopped turning.\n");

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
    /// @param errorRange The allowed tile errors from the target distance.
    /// @param runTimeout Maximum seconds the function will run for.
    void driveAndTurnDistanceTilesMotionProfile(double distanceTiles, double targetRotation, double maxVelocityPct, double errorRange, double runTimeout) {
        driveAndTurnDistanceWithInchesMotionProfile(distanceTiles * tileLengthIn, targetRotation, maxVelocityPct, errorRange * tileLengthIn, runTimeout);
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
        // driveAndTurnDistanceInchesMotionProfile(distanceInches, targetRotation, maxVelocityPct, errorRange, runTimeout);
        // return;
        
        // Variables
        double motorTargetDistanceRev = distanceInches / driveWheelCircumIn * (driveWheelMotorGearRatio);
        double leftMotorInitRev = LeftMotors.position(rev);
        double rightMotorInitRev = RightMotors.position(rev);

        // PID
        PIDControl driveTargetDistancePid(25, 0, 0, errorRange);
        PIDControl rotateTargetAnglePid(0.6);

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
            // printf("Turning.. Now: %.3f, Err: %.3f, Pid: %.3f\n", InertialSensor.rotation(deg), rotateError, rotateTargetAnglePid.getValue());

            // Compute final motor velocities
            double leftVelocityPct = velocityPct + rotateVelocityPct;
            double rightVelocityPct = velocityPct - rotateVelocityPct;
            
            // Scale percentages if overshoot
            double scaleFactor = 100.0 / fmax(100.0, fmax(fabs(leftVelocityPct), fabs(rightVelocityPct)));
            leftVelocityPct *= scaleFactor;
            rightVelocityPct *= scaleFactor;

            // Spin motors
            LeftMotors.spin(fwd, leftVelocityPct, pct);
            RightMotors.spin(fwd, rightVelocityPct, pct);

            // printf("Moving..\n");
            task::sleep(20);
        }
        printf("Stopped moving.\n");

        // Stop
        LeftRightMotors.stop(brake);
    }

    /// @brief Drive the robot for a specified distance in inches and rotate it to a specified rotation in degrees.
    /// @param distanceInches Distance in units of inches.
    /// @param targetRotation The target angle to face in degrees.
    /// @param maxVelocityPct Maximum velocity of the drive.
    /// @param errorRange The allowed inch errors from the target distance.
    /// @param runTimeout Maximum seconds the function will run for.
    void driveAndTurnDistanceWithInchesMotionProfile(double distanceInches, double targetRotation, double maxVelocityPct, double errorRange, double runTimeout) {
        // Variables
        double motorTargetDistanceRev = distanceInches / driveWheelCircumIn * (driveWheelMotorGearRatio);
        double leftMotorInitRev = LeftMotors.position(rev);
        double rightMotorInitRev = RightMotors.position(rev);

        // PID
        PIDControl driveTargetDistancePid(10, 0, 0, errorRange);
        PIDControl driveMaintainVelocityPid(2);
        PIDControl rotateTargetAnglePid(0.6);
        // Motion profile
        MotionProfile driveSpeedMotionProfile;
        driveSpeedMotionProfile.setModeAcceleration(90, 90, maxVelocityPct);
        driveSpeedMotionProfile.createProfile(motorTargetDistanceRev * driveMotorRevToPercentSecFactor);
        driveSpeedMotionProfile.start();
        printf(" --- Motion running for %.3f seconds. ---\n", driveSpeedMotionProfile.getMotionEndTime());

        timer timeout;
        while ((!driveSpeedMotionProfile.isDone() || !driveTargetDistancePid.isSettled() || !rotateTargetAnglePid.isSettled()) && timeout.value() < runTimeout) {
            // Compute average traveled motor revolution speeds
            double leftTravelVelocityPct = LeftMotors.velocity(pct);
            double rightTravelVelocityPct = RightMotors.velocity(pct);
            double averageTravelVelocityPct = (leftTravelVelocityPct + rightTravelVelocityPct) / 2;

            // Compute average traveled motor revolutions
            double leftTravelRev = LeftMotors.position(rev) - leftMotorInitRev;
            double rightTravelRev = RightMotors.position(rev) - rightMotorInitRev;
            double averageTravelRev = (leftTravelRev + rightTravelRev) / 2;

            // Compute velocity pid-value from error
            double distanceError = driveSpeedMotionProfile.getDistance() / (driveMotorRevToPercentSecFactor) - averageTravelRev;
            driveTargetDistancePid.computeFromError(distanceError);

            // Compute motor velocity from motion profile & pid-value
            // double velocityError = driveSpeedMotionProfile.getVelocity() - averageTravelVelocityPct;
            // driveMaintainVelocityPid.computeFromError(velocityError);
            // double velocityPct = fmin(100, fmax(-100, averageTravelVelocityPct + driveMaintainVelocityPid.getValue()));
            double velocityPct = driveSpeedMotionProfile.getVelocity() + driveTargetDistancePid.getValue();
            velocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, velocityPct));

            // Compute heading pid-value from error
            double rotateError = (targetRotation - InertialSensor.rotation());
            rotateTargetAnglePid.computeFromError(rotateError);
            double rotateVelocityPct = fmin(100, fmax(-100, rotateTargetAnglePid.getValue()));
            // printf("Turning.. Now: %.3f, Err: %.3f, Pid: %.3f\n", InertialSensor.rotation(deg), rotateError, rotateTargetAnglePid.getValue());

            // Compute final motor velocities
            double leftVelocityPct = velocityPct + rotateVelocityPct;
            double rightVelocityPct = velocityPct - rotateVelocityPct;
            
            // Scale percentages if overshoot
            double scaleFactor = 100.0 / fmax(100.0, fmax(fabs(leftVelocityPct), fabs(rightVelocityPct)));
            leftVelocityPct *= scaleFactor;
            rightVelocityPct *= scaleFactor;
            printf("MPVel: %.3f, AvgVel: %.3f, FinVel: %.3f\n", driveSpeedMotionProfile.getVelocity(), averageTravelVelocityPct, velocityPct);

            // Spin motors
            LeftMotors.spin(fwd, leftVelocityPct, pct);
            RightMotors.spin(fwd, rightVelocityPct, pct);

            // printf("Moving..\n");
            task::sleep(20);
        }
        printf("Stopped moving.\n");

        // Stop
        LeftRightMotors.stop(brake);
    }

    /// @brief Set the state of the intake.
    /// @param state Intaking: 1, off: 0, outtaking: -1.
    void setIntakeState(int state) {
        setIntakeResolveState(state);
    }

    /// @brief Set the flywheel's spinning speed to a specified revolutions per minute.
    /// @param rpm The velocity of the flywheel motor in rpm.
    void setFlywheelSpeedRpm(double rpm) {
        setFlywheelSpeed(rpm);
    }

    /// @brief Set the state of Left Wing's pneumatic.
    /// @param state Expanded: true, retracted: false.
    void setLeftWingState(bool state) {
        LeftWingPneumatic.set(state);
    }

    /// @brief Set the state of Right Wing's pneumatic.
    /// @param state Expanded: true, retracted: false.
    void setRightWingState(bool state) {
        RightWingPneumatic.set(state);
    }

    /// @brief Set the state of Left and Right Wing's pneumatic.
    /// @param state Expanded: true, retracted: false.
    void setWingsState(bool state) {
        setLeftWingState(state);
        setRightWingState(state);
    }

    /// @brief Set the state of the anchor's pneumatic.
    /// @param state Deployed: true, retracted: false
    void setAnchorState(bool state) {
        AnchorPneumatic.set(state);
    }

    /// @brief Set the state of the lift's pneumatic.
    /// @param state Lifted: true, lowered: false
    void setLiftState(bool state) {
        LiftPneumatic1.set(!state);
        LiftPneumatic2.set(!state);
    }    
}