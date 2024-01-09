#pragma once

enum controlType {
    ArcadeTwoStick,
    ArcadeTwoStickControlledTurn,
    ArcadeSingleStick,
    Mario
};

void preautonDrive();
void keybindDrive();
void controlDrive();