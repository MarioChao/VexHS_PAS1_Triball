#pragma once

void resetIntake();

void intakeThread();
void controlIntake();
void setIntakeResolveState(int intakeActivationState);
bool isIntakeControllable();