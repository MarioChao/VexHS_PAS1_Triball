#include "Mechanics/botFlywheel.h"
#include "Mechanics/botLift.h"
#include "Utilities/pidControl.h"
#include "main.h"

namespace {
    void resetFlywheelSpeed();
    void switchFlywheelSpeed();

    bool flywheelUsingPid = true;
    bool flywheelSpeedDebounce = false;
    
    int flywheelSpeedState = -1;
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
                printf("Flywheel motor settled\n");
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
    Controller1.ButtonRight.pressed([] () -> void {
        switchFlywheelSpeed();
    });
}
void setFlywheelSpeed(double rpm) {
    motAimSpeedRpm = rpm;
}

namespace {
    void resetFlywheelSpeed() {
        flywheelSpeedState = -1; // Inactive state
        setFlywheelSpeed(0);
    }

    void switchFlywheelSpeed() {
        if (!flywheelSpeedDebounce) {
            flywheelSpeedDebounce = true;

            // Increment speed state
            flywheelSpeedState++;
            flywheelSpeedState %= 2;

            // Spin flywheel
            switch (flywheelSpeedState) {
                case 0:
                    // Spin at max velocity
                    setFlywheelSpeed(600);
                    break;
                case 1:
                    // Reset flywheel speed
                    resetFlywheelSpeed();
                    break;
            }
            task::sleep(30);

            flywheelSpeedDebounce = false;
        }
    }
}