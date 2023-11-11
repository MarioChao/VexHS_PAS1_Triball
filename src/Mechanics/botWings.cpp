#include "Mechanics/botWings.h"
#include "main.h"

namespace {
    void changeBothWingsState();
    void changeLeftWingState();
    void changeRightWingState();
}

double wingsDebounce = false;

void keybindWings() {
    Controller1.ButtonR1.pressed([] () -> void {
        changeBothWingsState();
    });
}

namespace {
    void changeBothWingsState() {
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
}