#pragma once

void controlIntake();

/// @brief Set the intake On or Off, stopping it if the motor is stuck.
/// @param intakeActivated Activated: true, off: false.
void resolveIntake(bool intakeActivated);