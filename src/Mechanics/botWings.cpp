#include "Mechanics/botWings.h"
#include "Mechanics/botFlywheel.h"
#include "main.h"

namespace {
    void changeBothWingsState();
    void changeLeftWingState();
    void changeRightWingState();
}

double wingsDebounce = false;

void keybindWings() {
    Controller1.ButtonL1.pressed([] () -> void {
        changeBothWingsState();
    });
}
void setWingsState(bool state) {
    LeftWingPneumatic.set(state);
    RightWingPneumatic.set(state);
}

namespace {
    void changeBothWingsState() {
        if (!wingsDebounce) {
            wingsDebounce = true;

            int oldValue = LeftWingPneumatic.value();
            int newValue = oldValue ^ 1;
            setWingsState(newValue);
            if (newValue == true) {
                setFlywheelSpeed(460);
            } else {
                setFlywheelSpeed(0);
            }
            task::sleep(50);

            wingsDebounce = false;
        }
    }
    void changeLeftWingState() {
        if (!wingsDebounce) {
            wingsDebounce = true;

            int oldValue = LeftWingPneumatic.value();
            int newValue = oldValue ^ 1;
            LeftWingPneumatic.set(newValue);
            task::sleep(50);

            wingsDebounce = false;
        }
    }
    void changeRightWingState() {
        if (!wingsDebounce) {
            wingsDebounce = true;

            int oldValue = RightWingPneumatic.value();
            int newValue = oldValue ^ 1;
            RightWingPneumatic.set(newValue);
            task::sleep(50);

            wingsDebounce = false;
        }
    }
}