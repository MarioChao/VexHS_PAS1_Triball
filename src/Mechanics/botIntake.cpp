#include "Mechanics/botIntake.h"
#include "main.h"

namespace {
    void resolveIntake();
    bool intakeIsStopped();

    double intakeVelocityPct = 100.0;
    int intakeResolveState = 0;

    double intakeSkipMinFrameCount = 5;
    double intakeStuckFrameCount = 0;
    bool intakeIsStuck = false;
    
    bool canControlIntake = true;
}


void resetIntake() {
    intakeVelocityPct = 50.0;
    setIntakeResolveState(-1);
    task::sleep(100);
    setIntakeResolveState(0);
    intakeVelocityPct = 100.0;
}
void intakeThread() {
    // Intake loop
    intakeVelocityPct = 100.0;
    while (true) {
        resolveIntake();
        task::sleep(20);
    }
}
void controlIntake() {
    if (isIntakeControllable()) {
        intakeVelocityPct = 100.0;
        int intakeDirection = (int) Controller1.ButtonR1.pressing() - (int) Controller1.ButtonR2.pressing();
        setIntakeResolveState(intakeDirection);
    }
}
void setIntakeResolveState(int intakeActivationState) {
    intakeResolveState = intakeActivationState;
}
bool isIntakeControllable() {
    return canControlIntake;
}

namespace {
    /// @brief Set the intake On (intake or outtake) or Off, stopping it if the motor is stuck. Intake state is modified by setIntakeResolveState(int).
    void resolveIntake() {
        // Make sure intakeResolveState is within [-1, 1]
        intakeResolveState = (intakeResolveState > 0) - (intakeResolveState < 0);
        double spinVelocityPct = intakeResolveState * intakeVelocityPct;

        // Resolve intake
        if (intakeResolveState) {
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
    bool intakeIsStopped() {
        return fabs(IntakeMotor.velocity(pct)) < 20;
    }
}