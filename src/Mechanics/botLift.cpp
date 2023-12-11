#include "Mechanics/botLift.h"
#include "Mechanics/botFlywheel.h"
#include "main.h"

namespace {
    void resetLiftTask();

    void switchLiftState();
    void setLiftState(bool value);

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

namespace {
    /// @brief Reset lift's position. Used by calling resetLift().
    void resetLiftTask() {
        if (!liftDebounce) {
            liftDebounce = true;
            
            setLiftState(true);

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
    void setLiftState(bool value) {
        liftState = value;
        LiftPneumatic1.set(liftState);
        LiftPneumatic2.set(liftState);
    }
}