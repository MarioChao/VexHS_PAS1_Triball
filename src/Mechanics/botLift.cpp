#include "Mechanics/botLift.h"
#include "Mechanics/botFlywheel.h"
#include "main.h"

namespace {
    void resetLiftTask();

    void switchLiftState();
    void changeLiftMotorToDegree(double rotation, double liftVoltage = 9, double rotateTimeout = 3);

    void switchElevationState();
    void elevationNextState();
    void elevationLiftUpper();
    // void elevationLiftLowerClamp1();
    // void elevationLiftLowerClamp2();
    // void elevationLiftLowerClampFinal();
    void elevationLiftLowerHold();

    double rotateESP = 1e-9;
}

int liftState = 0;
int elevationState = 0;

bool liftResetted = false;
bool liftElevating = false;

bool liftDebounce = false;
bool elevationDebounce = false;

void preautonLift() {
    // Brake-types
    LiftMotor1.setStopping(coast);
    LiftMotor2.setStopping(coast);
}
void resetLift() {
    // Runs a task to reset the lift
    task liftTask([] () -> int {
        resetLiftTask();
        return 1;
    });
}
void keybindLift() {
    Controller1.ButtonUp.pressed([] () -> void {
        switchLiftState();
    });
}
void controlElevation() {
    bool elevationControlState = (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) || (Controller2.ButtonUp.pressing());
    if (elevationControlState) {
        liftElevating = true;
        switchElevationState();
    }
}

namespace {
    /// @brief Reset lift's position. Used by calling resetLift().
    void resetLiftTask() {
        if (!liftResetted && !liftDebounce) {
            liftDebounce = true;
            
            // Spin the lift upwards a bit
            LiftMotors.resetPosition();
            LiftMotors.spinTo(30, deg, false);
            task::sleep(100);

            // Spin the lift down for maximum 1 second
            timer timeout;
            timeout.reset();
            LiftMotors.spin(reverse, 50, pct);
            task::sleep(30);
            while (fabs(LiftMotor1.velocity(pct)) >= 20 && timeout.value() < 0.5) {
                task::sleep(10);
            }
            LiftMotors.stop();
            while (fabs(LiftMotor1.velocity(pct)) > 3) {
                task::sleep(10);
            }

            // Reset lift's position
            LiftMotors.resetPosition();

            // Completion
            liftResetted = true;
            liftDebounce = false;
        }
    }

    /// @brief Change the lift's position to high or low.
    void switchLiftState() {
        if (!liftDebounce && liftResetted && !liftElevating) {
            liftDebounce = true;

            liftState++;
            liftState %= 2;

            // Activate/deactivate flywheel
            switchFlywheelSpeed();

            // Change lift position
            switch (liftState) {
                case 0:
                    // Lower the lift
                    changeLiftMotorToDegree(0);
                    LiftMotor1.stop(coast);
                    break;
                case 1:
                    // Lift
                    changeLiftMotorToDegree(590);
                    LiftMotor1.stop(hold);
                    break;
            }
            task::sleep(10);

            liftDebounce = false;
        }
    }
    
    /// @brief Spin the lift motors to a certain angle in degrees.
    /// @param rotation The final angle (degrees) as read by LiftMotor1's encoder.
    /// @param liftVoltage The voltage to spin the lift motors.
    /// @param rotateTimeout The maximum number of seconds that the lift motors will rotate for.
    void changeLiftMotorToDegree(double rotation, double liftVoltage, double rotateTimeout) {
        canControlIntake = false;

        // Synchronize the two motor positions
        LiftMotor2.setPosition(LiftMotor1.position(deg), deg);
        // Calculate voltage with direction
        int spinDirection = (rotation > LiftMotors.position(deg)) - (rotation < LiftMotors.position(deg));
        double actualVoltage = fabs(liftVoltage) * spinDirection;
        // Spin to given rotation for maximum 3 seconds
        timer timeout;
        timeout.reset();
        LiftMotors.spin(fwd, actualVoltage, volt);
        bool reachedGoal = false;
        while (!reachedGoal && (rotateTimeout <= rotateESP || timeout.value() < rotateTimeout)) {
            if (spinDirection > 0 && LiftMotor1.position(deg) >= rotation) {
                reachedGoal = true;
            } else if (spinDirection < 0 && LiftMotor1.position(deg) <= rotation) {
                reachedGoal = true;
            }
            task::sleep(10);
        }
        LiftMotors.stop();

        canControlIntake = true;
    }

    void switchElevationState() {
        if (!elevationDebounce) {
            task elevationTask([] () -> int {
                elevationDebounce = true;
                elevationNextState();
                while ((Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) || (Controller2.ButtonUp.pressing())) {
                    task::sleep(10);
                }
                task::sleep(100);
                elevationDebounce = false;
                return 1;
            });
        }
    }
    void elevationNextState() {
        elevationState++;
        switch (elevationState) {
            case 1:
                elevationLiftUpper();
                break;
            case 2:
                LiftClampPneumatic.set(true);
            // case 2:
            //     elevationLiftLowerClamp1();
            //     break;
            // case 3:
            //     elevationLiftLowerClamp2();
            //     break;
            // case 4:
            //     elevationLiftLowerClampFinal();
            //     break;
            default:
                elevationLiftLowerHold();
                break;
        }
    }
    void elevationLiftUpper() {
        changeLiftMotorToDegree(450, 12);
        LiftMotor1.stop(hold);
    }
    // void elevationLiftLowerClamp1() {
    //     changeLiftMotorToDegree(250, 12);
    //     LiftMotor1.stop(coast);
    // }
    // void elevationLiftLowerClamp2() {
    //     changeLiftMotorToDegree(150, 12);
    //     LiftMotor1.stop(coast);
    // }
    // void elevationLiftLowerClampFinal() {
    //     changeLiftMotorToDegree(-100, 12, 0);
    //     LiftMotor1.stop(coast);
    // }
    void elevationLiftLowerHold() {
        canControlIntake = false;
        
        bool isHolding;
        isHolding = (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) || (Controller2.ButtonUp.pressing());
        LiftMotors.spin(fwd, -12, volt);
        while (isHolding) {
            isHolding = (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) || (Controller2.ButtonUp.pressing());
            task::sleep(10);
        }
        LiftMotor1.stop(hold);

        canControlIntake = true;
    }
}