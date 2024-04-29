#include "main.h"

/*
* Dimensions Note: 
* From center of wheel to center of wheel the robot is 10.5 x 10 & 3/8 inches.
* Border dimensions are 14.5 x 13 inches.
*/

// Drivetrain motor ports
const int LEFT_FRONT_WHEEL_PORT = 19;
const int LEFT_MIDDLE_WHEEL_PORT = 20; // Solid/Tracking Wheel
const int LEFT_BACK_WHEEL_PORT = 18;
const int RIGHT_FRONT_WHEEL_PORT = 12;
const int RIGHT_MIDDLE_WHEEL_PORT = 11; // Solid/Tracking Wheel
const int RIGHT_BACK_WHEEL_PORT = 13;

// Intake motor ports
const int LEFT_INTAKE_MOTOR_PORT = 9;
const int RIGHT_INTAKE_MOTOR_PORT = 2;

const int IMU_PORT = 3; // Inertial Measurement Unit

// Note: If motors have r10 board,
// do not go above 10,000mV and switch directions.
// We are trying to return and replace those motors.
const int VEX_MAX_VOLTAGE = 12000;
const int MAX_VOLTAGE = VEX_MAX_VOLTAGE - 0; // Use if you want to limit the potential voltage
const int ANALOG_MAX_VALUE = 127;
const double INTERPOLATION_MAGNITUDE = 0.01;
const int INTERPOLATION_ERROR = 30;
const double pi = 3.14159265358979323846;


//From the center of the wheel to the center of the other wheel
const double length = 10.5;
const double width = 10.375;

//The diameter of the wheel
const double diameter = 3.917;

//The amount the left side needs to catch up to the right
const double offset = 1.00;

// Global timer
int last_time = 0;

// Controller and motor setup
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_top_motor(LEFT_FRONT_WHEEL_PORT, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_middle_motor(LEFT_MIDDLE_WHEEL_PORT, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_bottom_motor(LEFT_BACK_WHEEL_PORT, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_top_motor(RIGHT_FRONT_WHEEL_PORT, pros::E_MOTOR_GEAR_200, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_middle_motor(RIGHT_MIDDLE_WHEEL_PORT, pros::E_MOTOR_GEAR_200, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_bottom_motor(RIGHT_BACK_WHEEL_PORT, pros::E_MOTOR_GEAR_200, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_intake_motor(LEFT_INTAKE_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_intake_motor(RIGHT_INTAKE_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, pros::E_MOTOR_ENCODER_DEGREES);

// Motor groups
pros::Motor_Group left_group({ left_top_motor, left_middle_motor, left_bottom_motor });
pros::Motor_Group right_group({ right_top_motor, right_middle_motor, right_bottom_motor });
pros::Motor_Group intake_group({ left_intake_motor, right_intake_motor });

// Inertial Sensor
pros::Imu imu_sensor(IMU_PORT);

// Pneumatic Piston
pros::ADIDigitalOut piston ('A');

// Global timer
double start_time;
double current_time;

double dabs(double v) {
	return v < 0.0 ? -v : v;
}

double to_radians(double degrees) {
	return pi * degrees / 180.;
}

void move(int voltage, double distance) {
	double v = left_middle_motor.get_position(); // if this is degrees
	const double iv = v;
	const double r = diameter / 2.;
	while (dabs(to_radians(iv - v) * r) < distance) {
		left_group.move_voltage(voltage);
		right_group.move_voltage(voltage);
		v = left_middle_motor.get_position();
		pros::lcd::print(0, "%f %f %f", iv, v, r);
	}
	left_group.brake();
	right_group.brake();
}

/**
 * This function is used to turn the robot a certain amount of degrees using the motor's built in encoders.
 * 
 * @param voltage: The voltage that the motors will be set to.
 * @param rotation: The amount of degrees that the robot will turn.
 */
void turn(int voltage, double rotation) {
    double initialIntertialRotation = imu_sensor.get_rotation();

	rotation *= 0.84;

	if (rotation > 0) {
		while (imu_sensor.get_rotation() - initialIntertialRotation < rotation) {
			right_group.move_voltage(-voltage);
			left_group.move_voltage(voltage);
		}
	} else {
		while (imu_sensor.get_rotation() - initialIntertialRotation > rotation) {
			right_group.move_voltage(voltage);
			left_group.move_voltage(-voltage);
		}
	}

	left_group.brake();
	right_group.brake();
}

void initialize() {
	//pros::lcd::initialize();
	pros::ADIDigitalOut piston ('A', true);
	left_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	right_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
}

void pushPull() {
	intake_group.move_relative(-990, 100);
	piston.set_value(false); // Punch
	// Back Up
	pros::delay(100);
	move(-2000, 5);
	//Ram
	left_group.move_voltage(8000);
	right_group.move_voltage(8000);
	piston.set_value(true); // Retrack
	pros::delay(1250); // Catch (Was 500)
	left_group.brake();
	right_group.brake();
	intake_group.move_relative(-90, 100); //shut flood gate
	pros::delay(500);
	intake_group.move_relative(90, 100); // open flood gate
}

void disabled() {}

void competition_initialize() {
	intake_group.move_absolute(0, 100); // Moves 100 units forward
	while (!((intake_group.get_positions().at(0) < 5) && (intake_group.get_positions().at(0) > -5))) {
		pros::delay(5);
	}
}

void autonomous() {
	start_time = pros::millis();
	move(-9500, 40.5);
	turn(3000, 77);
	move(12000, 43.5);
	turn(3000, 90);
	piston.set_value(true);
	intake_group.move_relative(-180, 200);
	pros::delay(500);
	for (int i = 0; i < 20; i++) {
		pushPull();
	}
}

void opcontrol() {
	// Arcade Drive
	while (true) {
		// Joystick input
		int x = master.get_analog(ANALOG_RIGHT_X);
		int y = master.get_analog(ANALOG_LEFT_Y);

		if (x == 0 && y == 0) {
			// Stop motors if joystick is at the center
			// interpolate_motor_voltage(left_group, 0);
			// interpolate_motor_voltage(right_group, 0);
			left_group.move_voltage(0);
			right_group.move_voltage(0);
			//pros::lcd::print(0, "%d %d %d", 0, 0, 0); // LCD display
		}
		else {
			// Calculate magnitude based on normalized x and y
			double normalized_x = (double) x / 127.0;
			double normalized_y = (double) y / 127.0;
			double magnitude = sqrt(normalized_x * normalized_x + normalized_y * normalized_y);

			// Calculate motor voltages based on joystick input
			double angle = atan2(y, x);
			double voltage_x = cos(angle) * MAX_VOLTAGE * magnitude;
			double voltage_y = sin(angle) * MAX_VOLTAGE * magnitude;

			// Distribute voltages for forward/backward and turning
			int voltage_left = voltage_y + voltage_x;
			int voltage_right = voltage_y - voltage_x;

			// Apply interpolated motor voltages
			left_group.move_voltage(voltage_left);
			right_group.move_voltage(voltage_right);
		}

		// Intake control
		if (master.get_digital(DIGITAL_R1)) { // Intake
			intake_group.move_voltage(6500);
		}
		else if (master.get_digital(DIGITAL_L1)) { // Release
			intake_group.move_voltage(-6500);
		}
		else {
			//Potentially change to move to absolute position if button is not being pressed
			intake_group.move_voltage(0);
		}

		// Piston control
		if (master.get_digital(DIGITAL_L2)) {
			piston.set_value(true);
		}
		else if (master.get_digital(DIGITAL_R2)){
			piston.set_value(false);
		}

		pros::delay(20); // pros::delay for loop iteration
	}
}
