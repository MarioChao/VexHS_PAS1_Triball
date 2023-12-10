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
        int intakeDirection = (int) Controller1.ButtonR1.pressing() - (int) Controller1.ButtonR2.pressing();
        if (intakeDirection == 1) {
            resolveIntake(true);
        } else if (intakeDirection == -1) {
            resolveIntake(-1);
        } else {
            resolveIntake(false);
        }
    }
}

void resolveIntake(int intakeActivationState) {
    // Make sure intakeActivationState is within [-1, 1]
    intakeActivationState = (intakeActivationState > 0) - (intakeActivationState < 0);
    double spinVelocityPct = intakeActivationState * 100;

    // Resolve intake
    if (intakeActivationState) {
        // Stop condition: intake is considered "stuck"
        if (intakeIsStuck) {
            IntakeMotor.stop();
            return;
        }

        // Spin the intake
        IntakeMotor.spin(fwd, spinVelocityPct, pct);
        
        // Update stuck state
        if (intakeIsStopped()) {
            // Intake is "stuck" if it's stopped for several frames
            if (intakeStuckFrameCount >= intakeSkipMinFrameCount) {
                intakeIsStuck = true;
            }
            intakeStuckFrameCount++;
            intakeStuckFrameCount = fmin(intakeStuckFrameCount, intakeSkipMinFrameCount + 1);
        } else {
            // Reset stuck state when intake is no longer stuck
            intakeIsStuck = false;
            intakeStuckFrameCount = 0;
        }
    } else {
        // Stop the intake
        IntakeMotor.stop();
        // Reset stuck state
        intakeIsStuck = false;
        intakeStuckFrameCount = 0;
    }
}

namespace {
    bool intakeIsStopped() {
        return fabs(IntakeMotor.velocity(pct)) < 20;
    }
}