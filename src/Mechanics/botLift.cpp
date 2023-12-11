#include "Mechanics/botLift.h"
#include "Mechanics/botIntake.h"
#include "main.h"

namespace {
    void resetLiftTask();

    void switchLiftState();

    bool liftDebounce = false;

    int liftState = 0;
}

void resetLift() {
    // Runs a task to reset the lift
    task liftTask([] () -> int {
        resetLiftTask();
        return 1;
    });
}
void keybindLift() {
    Controller1.ButtonB.pressed([] () -> void {
        switchLiftState();
    });
}
void setLiftState(bool value) {
    liftState = value;
    LiftPneumatic1.set(liftState);
    LiftPneumatic2.set(liftState);
}

namespace {
    /// @brief Reset lift's position. Used by calling resetLift().
    void resetLiftTask() {
        if (!liftDebounce) {
            liftDebounce = true;
            
            // Reset lift code here...

            liftDebounce = false;
        }
    }

    /// @brief Change the lift's position to high or low.
    void switchLiftState() {
        if (!liftDebounce) {
            liftDebounce = true;

            setLiftState(liftState ^ 1);
            task::sleep(10);

            liftDebounce = false;
        }
    }
}