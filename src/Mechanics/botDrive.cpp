#include "Mechanics/botDrive.h"
#include "Utilities/pidControl.h"
#include "Utilities/robotInfo.h"
#include "main.h"

namespace {
    using namespace botinfo;

    void switchDriveMode();
    void controlArcadeTwoStick();
    void controlArcadeTwoStickControlledTurn();
    void controlArcadeSingleStick();
    void controlMario();
    void drive(double initLeftPct, double initRightPct, double initPolarRotatePct, double rotateCenterOffsetIn = 0);

    // Drive mode
    controlType driveMode = controlType::ArcadeTwoStick;
    bool driveModeDebounce = false;

    // Mario drive
    PIDControl marioDriveDeltaVelocityPid(0.2, 0, 0); // Maintain velocity
    double marioVelocityPct = 0;
}

void preautonDrive() {
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);
}

void keybindDrive() {
    Controller1.ButtonX.pressed([] () -> void {
        switchDriveMode();
    });
}

void controlDrive() {
    switch (driveMode) {
        case controlType::ArcadeTwoStick:
            controlArcadeTwoStick();
            break;
        case controlType::ArcadeTwoStickControlledTurn:
            controlArcadeTwoStickControlledTurn();
            break;
        case controlType::ArcadeSingleStick:
            controlArcadeSingleStick();
            break;
        case controlType::Mario:
            controlMario();
            break;
        default:
            break;
    }
}


namespace {
    /// @brief Switch driving mode between arcade two stick, arcade single stick, and Mario
    void switchDriveMode() {
        if (!driveModeDebounce) {
            driveModeDebounce = true;

            switch (driveMode) {
                case controlType::ArcadeTwoStick:
                    driveMode = controlType::ArcadeTwoStickControlledTurn;
                    break;
                case controlType::ArcadeTwoStickControlledTurn:
                    driveMode = controlType::ArcadeTwoStick;
                    break;
                case controlType::ArcadeSingleStick:
                    driveMode = controlType::Mario;
                    break;
                case controlType::Mario:
                    driveMode = controlType::ArcadeTwoStick;
                    break;
                default:
                    break;
            }
            task::sleep(100);

            driveModeDebounce = false;
        }
    }

    /// @brief Drive in arcade mode (Axis3 forward/backward, Axis1 rotation)
    void controlArcadeTwoStick() {
        double axis3 = Controller1.Axis3.position();
        if (fabs(axis3) < 2) axis3 = 0;
        double axis1 = Controller1.Axis1.position();
        if (fabs(axis1) < 2) axis1 = 0;
        drive(axis3, axis3, -axis1 * 0.9);
    }

    void controlArcadeTwoStickControlledTurn() {
        double axis3 = Controller1.Axis3.position();
        if (fabs(axis3) < 2) axis3 = 0;
        double axis1 = Controller1.Axis1.position();
        if (fabs(axis1) < 2) axis1 = 0;

        // Off-center rotation (modified from Mario drive)
        double rotateYaw = axis1;
        double rotateFactor = (1 - fmin(fabs(rotateYaw), 100.0) / 100.0) * 1.5; // Bigger the axis1, smaller the center offset (more turn)
        double rotateSign = (rotateYaw > 0) - (rotateYaw < 0);
        double speedFactor = fmin(fabs(axis3 / 100.0), 100.0) * 0.6; // Greater the speed, greater the center offset (less turn)
        double centerOffset = robotLengthIn * rotateSign * rotateFactor * speedFactor;

        drive(axis3, axis3, -axis1, centerOffset);
    }

    /// @brief Drive in arcade mode (Axis3 forward/backward, Axis4 rotation)
    void controlArcadeSingleStick() {
        double axis3 = Controller1.Axis3.position();
        if (fabs(axis3) < 2) axis3 = 0;
        double axis4 = Controller1.Axis4.position();
        if (fabs(axis4) < 2) axis4 = 0;
        drive(axis3, axis3, -axis4);
    }

    /// @brief Drive in Mario mode (ButtonA forward, ButtonB backward, Axis3 rotation)
    void controlMario() {
        int driveDirection = Controller1.ButtonA.pressing() - Controller1.ButtonB.pressing();

        // Compute velocity based on Pid
        double velocityError;
        switch (driveDirection) {
            case 1:
                velocityError = 100 - marioVelocityPct;
                break;
            case -1:
                velocityError = -100 - marioVelocityPct;
                break;
            default:
                velocityError = 0 - marioVelocityPct;
        }
        marioDriveDeltaVelocityPid.computeFromError(velocityError);
        marioVelocityPct += marioDriveDeltaVelocityPid.getValue();

        // Off-center rotation
        double rotateYaw = Controller1.Axis4.position();
        double rotateFactor = (1 - fmin(fabs(rotateYaw), 100.0) / 100.0) * 1.5; // Bigger the axis4, smaller the center offset
        double rotateSign = (rotateYaw > 0) - (rotateYaw < 0);
        double speedFactor = fmin(fabs(marioVelocityPct / 100.0), 100.0) * 0.6; // Greater the speed, greater the center offset
        double centerOffset = robotLengthIn * rotateSign * rotateFactor * speedFactor;

        // Drive with velocity
        drive(marioVelocityPct, marioVelocityPct, -Controller1.Axis4.position(), centerOffset);
    }

    void drive(double initLeftPct, double initRightPct, double initPolarRotatePct, double rotateCenterOffsetIn) {
        // Compute scaled rotations
        double leftRotateRadiusIn = halfRobotLengthIn + rotateCenterOffsetIn;
        double rightRotateRadiusIn = halfRobotLengthIn - rotateCenterOffsetIn;
        double leftPolarRotatePct = initPolarRotatePct * (leftRotateRadiusIn / halfRobotLengthIn);
        double rightPolarRotatePct = initPolarRotatePct * (rightRotateRadiusIn / halfRobotLengthIn);

        // Compute final percentages
        double leftPct = initLeftPct - leftPolarRotatePct;
        double rightPct = initRightPct + rightPolarRotatePct;

        // Scale percentages if overshoot
        double scaleFactor = 100.0 / fmax(100.0, fmax(fabs(leftPct), fabs(rightPct)));
        leftPct *= scaleFactor;
        rightPct *= scaleFactor;

        // Spin motors at volt
        // LeftMotors.spin(fwd, leftPct, pct);
        // RightMotors.spin(fwd, rightPct, pct);
        if (fabs(leftPct) < 5) {
            LeftMotors.stop();
        } else {
            LeftMotors.spin(fwd, 12.0 * (leftPct / 100.0), volt);
        }
        if (fabs(rightPct) < 5) {
            RightMotors.stop();
        } else {
            RightMotors.spin(fwd, 12.0 * (rightPct / 100.0), volt);
        }
    }
}