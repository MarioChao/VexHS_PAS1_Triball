#include "Mechanics/botLift.h"
#include "Mechanics/botFlywheel.h"
#include "main.h"

namespace {
    void resetLiftTask();

    void switchLiftState();

    bool liftDebounce = false;
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

namespace {
    /// @brief Reset lift's position. Used by calling resetLift().
    void resetLiftTask() {
        if (!liftDebounce) {
            liftDebounce = true;
            
            LiftPneumatic.set(1);

            liftDebounce = false;
        }
    }

    /// @brief Change the lift's position to high or low.
    void switchLiftState() {
        if (!liftDebounce) {
            liftDebounce = true;

            int oldValue = LiftPneumatic.value();
            int newValue = oldValue ^ 1;
            LiftPneumatic.set(newValue);
            task::sleep(10);

            liftDebounce = false;
        }
    }
}