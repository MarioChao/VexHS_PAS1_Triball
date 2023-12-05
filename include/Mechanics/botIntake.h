#pragma once

void controlIntake();

/// @brief Set the intake On (intake or outtake) or Off, stopping it if the motor is stuck.
/// @param intakeActivationState Intaking: 1, off: 0, outtaking: -1.
void resolveIntake(int intakeActivationState);