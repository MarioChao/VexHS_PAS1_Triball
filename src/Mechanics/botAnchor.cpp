#include "Mechanics/botAnchor.h"
#include "main.h"

namespace {
    void changeAnchorState();
    
    bool anchorDebounce = false;
}


void keybindAnchor() {
    Controller1.ButtonY.pressed([] () -> void {
        changeAnchorState();
    });
}

namespace {
    void changeAnchorState() {
        if (!anchorDebounce) {
            anchorDebounce = true;

            int oldValue = AnchorPneumatic.value();
            int newValue = oldValue ^ 1;
            AnchorPneumatic.set(newValue);
            task::sleep(50);

            anchorDebounce = false;
        }
    }
}