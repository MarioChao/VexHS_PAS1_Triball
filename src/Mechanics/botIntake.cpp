#include "Mechanics/botIntake.h"
#include "main.h"

namespace {
    bool intakeIsStopped();

    double intakeSkipMinFrameCount = 5;
    double intakeStuckFrameCount = 0;
    bool intakeIsStuck = false;
}

void controlIntake() {
    if (canControlIntake) {
        int intakeDirection = (int) Controller1.ButtonR1.pressing();
        if (intakeDirection == 1) {
            resolveIntake(true);
        } else {
            resolveIntake(false);
        }
    }
}

void resolveIntake(bool intakeActivated) {
    if (intakeActivated) {
        // Stop condition: intake is considered "stuck"
        if (intakeIsStuck) {
            LiftMotor2.stop();
            return;
        }

        // Spin the intake
        LiftMotor2.spin(fwd, 80, pct);
        
        // Update stuck state
        if (intakeIsStopped()) {
            // Intake is "stuck" if it's stopped for several frames
            if (intakeStuckFrameCount >= intakeSkipMinFrameCount) {
                intakeIsStuck = true;
            }
            intakeStuckFrameCount++;
        } else {
            // Reset stuck state when intake is no longer stuck
            intakeIsStuck = false;
            intakeStuckFrameCount = 0;
        }
    } else {
        // Stop the intake
        LiftMotor2.stop();
        // Reset stuck state
        intakeIsStuck = false;
        intakeStuckFrameCount = 0;
    }
}

namespace {
    bool intakeIsStopped() {
        return LiftMotor2.velocity(pct) < 20;
    }
}