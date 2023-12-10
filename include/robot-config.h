using namespace vex;

extern brain Brain;

// Devices
extern controller Controller1;
extern controller Controller2;

extern motor_group LeftMotors;
extern motor_group RightMotors;
extern motor_group LeftRightMotors;
extern motor IntakeMotor;
extern motor FlywheelMotor;

extern pneumatics LeftWingPneumatic;
extern pneumatics RightWingPneumatic;
extern pneumatics AnchorPneumatic;
extern pneumatics LiftPneumatic;

extern inertial InertialSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
