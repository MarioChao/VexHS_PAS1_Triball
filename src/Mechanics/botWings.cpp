#include "Mechanics/botWings.h"
#include "main.h"

namespace {
    void changeFrontWingsState();
    void changeBothWingsToSameState();
    void changeBothWingsToDifferentState();
    void changeLeftWingState();
    void changeRightWingState();
    
    double frontWingsDebounce = false;
    double wingsDebounce = false;
    double leftWingDebounce = false;
    double rightWingDebounce = false;
}

void keybindWings() {
    Controller1.ButtonL1.pressed([] () -> void {
        // changeBothWingsToSameState();
        changeFrontWingsState();
    });
    Controller1.ButtonL2.pressed([] () -> void {
        changeLeftWingState();
    });
    Controller1.ButtonR2.pressed([] () -> void {
        changeRightWingState();
    });
}
void setWingsState(bool state) {
    LeftWingPneumatic.set(state);
    RightWingPneumatic.set(state);
}

namespace {
    void changeFrontWingsState() {
        if (!frontWingsDebounce) {
            frontWingsDebounce = true;

            int oldValue = FrontWingsPneumatic.value();
            int newValue = oldValue ^ 1;
            FrontWingsPneumatic.set(newValue);
            task::sleep(50);

            frontWingsDebounce = false;
        }
    }
    void changeBothWingsToSameState() {
        if (!wingsDebounce) {
            wingsDebounce = true;

            int oldValue = LeftWingPneumatic.value() && RightWingPneumatic.value();
            int newValue = oldValue ^ 1;
            setWingsState(newValue);
            task::sleep(50);

            wingsDebounce = false;
        }
    }
    void changeBothWingsToDifferentState() {
        if (!wingsDebounce) {
            wingsDebounce = true;

            int oldValue1 = LeftWingPneumatic.value();
            int oldValue2 = RightWingPneumatic.value();
            int newValue1 = oldValue1 ^ 1;
            int newValue2 = oldValue2 ^ 1;
            LeftWingPneumatic.set(newValue1);
            RightWingPneumatic.set(newValue2);
            task::sleep(50);

            wingsDebounce = false;
        }
    }
    void changeLeftWingState() {
        if (!leftWingDebounce) {
            leftWingDebounce = true;

            int oldValue = LeftWingPneumatic.value();
            int newValue = oldValue ^ 1;
            LeftWingPneumatic.set(newValue);
            task::sleep(50);

            leftWingDebounce = false;
        }
    }
    void changeRightWingState() {
        if (!rightWingDebounce) {
            rightWingDebounce = true;

            int oldValue = RightWingPneumatic.value();
            int newValue = oldValue ^ 1;
            RightWingPneumatic.set(newValue);
            task::sleep(50);

            rightWingDebounce = false;
        }
    }
}