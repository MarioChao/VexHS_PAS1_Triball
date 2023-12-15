#pragma once

void resetIntake();

void intakeThread();
void controlIntake();
void setIntakeResolveState(int intakeActivationState, double newIntakeVelocityPct = 100.0);
bool isIntakeControllable();