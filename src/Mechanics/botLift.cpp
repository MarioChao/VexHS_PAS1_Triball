#include "Mechanics/botLift.h"
#include "main.h"

namespace {
    void switchLiftState();

    bool liftDebounce = false;

    int liftState = 0;
}

void keybindLift() {
    Controller1.ButtonB.pressed([] () -> void {
        switchLiftState();
    });
}
void setLiftState(bool value) {
    liftState = value;
    LiftPneumatic.set(liftState);
}

namespace {
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