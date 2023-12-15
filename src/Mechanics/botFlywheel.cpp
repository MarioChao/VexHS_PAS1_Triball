#include "Mechanics/botFlywheel.h"
#include "Utilities/pidControl.h"
#include "Utilities/debugFunctions.h"
#include "main.h"

namespace {
    void resetFlywheelSpeed();
    void switchFlywheelSpeed(double rpm);
    void decreaseFlywheelSpeed();
    void showFlywheelSpeedOnController();

    double flywheelBangToleranceRpm = 50.0;
    double flywheelTermVoltageCoefficient = 0.975;
    double flywheelToleranceDVolt = 5.0;
    bool usingCustomTuning = false;
    bool flywheelSpeedDebounce = false;
}

void flywheelThread() {
    // Variables
    double error;
    double termVoltage;
    double oldVoltage, voltError, deltaVoltage;
    double newVoltage, modifiedVoltage;
    // Flywheel velocity loop
    while (true) {
        motSpeedRpm = FlywheelMotor.velocity(rpm);
        if (usingCustomTuning) {
            // Compute RPM error
            error = motAimSpeedRpm - motSpeedRpm;

            // Compute the target voltage
            termVoltage = motAimSpeedRpm / 600.0 * 12.0 * flywheelTermVoltageCoefficient;

            // Compute new voltage
            oldVoltage = FlywheelMotor.voltage();
            voltError = termVoltage - oldVoltage;
            deltaVoltage = fmin(flywheelToleranceDVolt, fmax(-flywheelToleranceDVolt, voltError));
            newVoltage = oldVoltage + deltaVoltage;
            newVoltage = fmin(12, fmax(-12, newVoltage));
            
            // Compute modified new voltage
            modifiedVoltage = 0;
            if (fabs(motAimSpeedRpm) >= 590) {
                // Special case: max RPM
                double spinDirection = (motAimSpeedRpm > 0) - (motAimSpeedRpm < 0);
                modifiedVoltage = spinDirection * 12;
            } else if (fabs(error) > flywheelBangToleranceRpm) {
                // Special case: high error
                // Bang-bang Control
                if (motAimSpeedRpm > 0) {
                    if (error > 0) {
                        modifiedVoltage = 12;
                    } else {
                        modifiedVoltage = 0;
                    }
                } else if (motAimSpeedRpm < 0) {
                    if (error < 0) {
                        modifiedVoltage = -12;
                    } else {
                        modifiedVoltage = 0;
                    }
                }
            } else {
                modifiedVoltage = newVoltage;
            }

            // Spin the flywheel
            if (fabs(motAimSpeedRpm) < 20) {
                // Special case: low RPM
                // Coast the flywheel
                FlywheelMotor.stop(coast);
            } else if (fabs(modifiedVoltage - oldVoltage) <= flywheelToleranceDVolt) {
                // Spin at the modified voltage
                FlywheelMotor.spin(fwd, modifiedVoltage, volt);
            } else {
                // Spin at the computed voltage
                FlywheelMotor.spin(fwd, newVoltage, volt);
            }
        } else {
            FlywheelMotor.spin(fwd, motAimSpeedRpm, rpm);
        }

        task::sleep(30);
    }
}
void keybindFlywheel() {
    Controller1.ButtonUp.pressed([] () -> void {
        switchFlywheelSpeed(460);
    });
    Controller1.ButtonDown.pressed([] () -> void {
        switchFlywheelSpeed(-460);
    });
    Controller1.ButtonY.pressed([] () -> void {
        decreaseFlywheelSpeed();
    });
}
void setFlywheelSpeed(double rpm) {
    if (!flywheelSpeedDebounce) {
        flywheelSpeedDebounce = true;
        motAimSpeedRpm = rpm;
        showFlywheelSpeedOnController();
        flywheelSpeedDebounce = false;
    }
}

namespace {
    void resetFlywheelSpeed() {
        setFlywheelSpeed(0);
    }
    void switchFlywheelSpeed(double rpm) {
        if (motAimSpeedRpm == rpm) {
            // Deactivate
            setFlywheelSpeed(0);
        } else {
            setFlywheelSpeed(rpm);
        }
    }
    void decreaseFlywheelSpeed() {
        if (!flywheelSpeedDebounce) {
            flywheelSpeedDebounce = true;
            motAimSpeedRpm -= 50;
            showFlywheelSpeedOnController();
            flywheelSpeedDebounce = false;
        }
    }
    void showFlywheelSpeedOnController() {
        std::string speedStr = "";
        int tmpSpeed = (int) motAimSpeedRpm;
        while (tmpSpeed) {
            speedStr = (char) ('0' + tmpSpeed % 10) + speedStr;
            tmpSpeed /= 10;
        }
        debug::printOnController("Flywheel at " + speedStr + " rpm");
    }
}