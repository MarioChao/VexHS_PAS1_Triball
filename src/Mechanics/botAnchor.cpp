#include "Mechanics/botAnchor.h"
#include "main.h"

void changeAnchorState();

double anchorDebounce = false;

void keybindAnchor() {
    Controller1.ButtonY.pressed([] () -> void {
        changeAnchorState();
    });
}

void changeAnchorState() {
    if (!anchorDebounce) {
        anchorDebounce =  true;

        int oldValue = AnchorPneumatic.value();
        int newValue = oldValue ^ 1;
        AnchorPneumatic.set(newValue);
        task::sleep(50);

        anchorDebounce = false;
    }
}