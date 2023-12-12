#include "Mechanics/botFlywheel.h"
#include "Utilities/pidControl.h"
#include "main.h"

namespace {
    void resetFlywheelSpeed();
    void switchFlywheelSpeed(double rpm);

    bool flywheelUsingPid = true;
    bool flywheelSpeedDebounce = false;
}

void flywheelThread() {
    // Variables
    PIDControl flywheelDeltaVelocityPid(0.04, 0, 0); // Maintain velocity
    double oldVoltage, pidValue;
    double newVoltage;
    bool alreadySettled = false;
    // Flywheel velocity loop
    while (true) {
        motSpeedRpm = FlywheelMotor.velocity(rpm);
        if (flywheelUsingPid) {
            // Compute PID value from error
            flywheelDeltaVelocityPid.computeFromError(motAimSpeedRpm - motSpeedRpm);

            // Compute the needed voltage
            oldVoltage = FlywheelMotor.voltage(volt);
            pidValue = flywheelDeltaVelocityPid.getValue();
            // Settled debug
            if (flywheelDeltaVelocityPid.isSettled() && !alreadySettled) {
                // printf("Flywheel motor settled\n");
                alreadySettled = true;
            } else if (!flywheelDeltaVelocityPid.isSettled()) {
                alreadySettled = false;
            }
            newVoltage = fmin(12, fmax(-12, oldVoltage + pidValue));
            
            // Spin the flywheel
            if (fabs(motAimSpeedRpm) < 20) {
                // Special case: low RPM
                FlywheelMotor.stop(coast);
                flywheelDeltaVelocityPid.setErrorI(0);
            } else if (fabs(motAimSpeedRpm) >= 590) {
                // Special case: max RPM
                double spinDirection = (motAimSpeedRpm > 0) - (motAimSpeedRpm < 0);
                FlywheelMotor.spin(fwd, spinDirection * 12, volt);
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
        switchFlywheelSpeed(550);
    });
    Controller1.ButtonDown.pressed([] () -> void {
        switchFlywheelSpeed(-550);
    });
}
void setFlywheelSpeed(double rpm) {
    motAimSpeedRpm = rpm;
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
}