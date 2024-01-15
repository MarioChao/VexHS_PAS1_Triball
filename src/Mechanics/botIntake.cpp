#include "Mechanics/botIntake.h"
#include "main.h"

namespace {
    void resolveIntake();

    int intakeResolveState = 0;
    
    bool canControlIntake = true;
}

void intakeThread() {
    // Intake loop
    while (true) {
        resolveIntake();
        task::sleep(20);
    }
}
void controlIntake() {
    if (isIntakeControllable()) {
        int intakeDirection = (int) Controller1.ButtonR1.pressing();
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
    /// @brief Set the intake to Holding (0) or Released (1). Intake state is modified by setIntakeResolveState(int).
    void resolveIntake() {
        // Make sure intakeResolveState is within [0, 1]
        intakeResolveState = (intakeResolveState > 0);
        
        // Check if intake state is already reached
        if (intakeResolveState == IntakePneumatic.value()) {
            return;
        }

        // Resolve intake
        if (intakeResolveState) {
            // Released
            IntakePneumatic.set(true);
        } else {
            // Hold
            IntakePneumatic.set(false);
        }
    }
}