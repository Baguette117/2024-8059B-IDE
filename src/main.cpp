#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	Motor leftFront(leftFrontPort, leftFrontGearset, leftFrontEncoder);
    Motor leftMid(leftMidPort, leftMidGearset, leftMidEncoder);
    Motor leftBack(leftBackPort, leftBackGearset, leftBackEncoder);
    Motor rightFront(rightFrontPort, rightFrontGearset, rightFrontEncoder);
    Motor rightMid(rightMidPort, rightMidGearset, rightMidEncoder);
    Motor rightBack(rightBackPort, rightBackGearset, rightBackEncoder);
	Motor cata(cataPort, cataGearset, cataEncoder);
	Motor intake(intakePort, intakeGearset, intakeEncoder);
	adi::DigitalOut wingLeft(wingLeftPort);
	adi::DigitalOut wingRight(wingRightPort);
	Imu inertial(inertialPort);
	Controller master(CONTROLLER_MASTER);

	inertial.reset();
	master.clear();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// calibration(0);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Motor leftFront(leftFrontPort);
    Motor leftMid(leftMidPort);
    Motor leftBack(leftBackPort);
    Motor rightFront(rightFrontPort);
    Motor rightMid(rightMidPort);
    Motor rightBack(rightBackPort);
	Motor cata(cataPort);
	Motor intake(intakePort);
	adi::DigitalOut wingLeft(wingLeftPort);
	adi::DigitalOut wingRight(wingRightPort);
    Imu inertial(inertialPort);
	Controller master(CONTROLLER_MASTER);

	bool invert = false, wingLeftState = false, wingRightState = false, elevState = false;
	double left, right;

	leftFront.set_brake_mode(MOTOR_BRAKE_HOLD);
	leftMid.set_brake_mode(MOTOR_BRAKE_HOLD);
	leftBack.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightFront.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightMid.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightBack.set_brake_mode(MOTOR_BRAKE_HOLD);

    while (true) {
		left = 1.27*master.get_analog(ANALOG_LEFT_Y);
		right = 1.27*master.get_analog(ANALOG_RIGHT_Y);

		if (invert){
			leftFront.move(-right);
			leftMid.move(-right);
			leftBack.move(-right);
			rightFront.move(-left);
			rightMid.move(-left);
			rightBack.move(-left);
		} else {
			leftFront.move(left);
			leftMid.move(left);
			leftBack.move(left);
			rightFront.move(right);
			rightMid.move(right);
			rightBack.move(right);
		}

		if (master.get_digital_new_press(DIGITAL_UP)){
			shoot();
		}

		if (master.get_digital(DIGITAL_L1)){
			intake.move(-80);
		} else if (master.get_digital(DIGITAL_R1)) {
			intake.move(127);
		} else {
			intake.move(0);
		}

		wingLeft.set_value(master.get_digital(DIGITAL_R2));
		wingRight.set_value(master.get_digital(DIGITAL_L2));

		if (master.get_digital_new_press(DIGITAL_UP)){
			invert = !invert;
		}
        delay(20);
    }
}
