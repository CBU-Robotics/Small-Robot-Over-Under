#include "main.h"

/*
* Dimensions Note: 
* From center of wheel to center of wheel the robot is 10.5 x 10 & 3/8 inches.
* Border dimensions are 14.5 x 13 inches.
*/

// Drivetrain motor ports
const int LEFT_TOP_MOTOR_PORT = 19;
const int LEFT_Middle_MOTOR_PORT = 20; // Middle Wheel
const int LEFT_BOTTOM_MOTOR_PORT = 18;
const int RIGHT_TOP_MOTOR_PORT = 12;
const int RIGHT_Middle_MOTOR_PORT = 11; // Middle Wheel
const int RIGHT_BOTTOM_MOTOR_PORT = 13;

// Intake motor ports
const int LEFT_INTAKE_MOTOR_PORT = 9;
const int RIGHT_INTAKE_MOTOR_PORT = 2;

const int IMU_PORT = 8; // Inertial Measurement Unit

// Note: If motors have r10 board,
// do not go above 10,000mV and switch directions.
// We are trying to return and replace those motors.
const int VEX_MAX_VOLTAGE = 12000;
const int MAX_VOLTAGE = VEX_MAX_VOLTAGE - 0;
const int ANALOG_MAX_VALUE = 127;
const double INTERPOLATION_MAGNITUDE = 0.01;
const int INTERPOLATION_ERROR = 30;
const double pi = 3.14159265358979323846;


//From the center of the wheel to the center of the other wheel
const double length = 10.5;
const double width = 10.375;

//The diameter of the wheel
const double diameter = 4.0;

//The amount the left side needs to catch up to the right
const double offset = 1.00;

// Controller and motor setup
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_top_motor(LEFT_TOP_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, MOTOR_ENCODER_ROTATIONS);
pros::Motor left_middle_motor(LEFT_Middle_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, MOTOR_ENCODER_ROTATIONS);
pros::Motor left_bottom_motor(LEFT_BOTTOM_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, MOTOR_ENCODER_ROTATIONS);
pros::Motor right_top_motor(RIGHT_TOP_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, MOTOR_ENCODER_ROTATIONS);
pros::Motor right_middle_motor(RIGHT_Middle_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, MOTOR_ENCODER_ROTATIONS);
pros::Motor right_bottom_motor(RIGHT_BOTTOM_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, MOTOR_ENCODER_ROTATIONS);
pros::Motor left_intake_motor(LEFT_INTAKE_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, MOTOR_ENCODER_ROTATIONS);
pros::Motor right_intake_motor(RIGHT_INTAKE_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, MOTOR_ENCODER_ROTATIONS);

// Motor groups and brake modes
pros::Motor_Group left_group({ left_top_motor, left_middle_motor, left_bottom_motor });
pros::Motor_Group right_group({ right_top_motor, right_middle_motor, right_bottom_motor });
pros::Motor_Group intake_group({ left_intake_motor, right_intake_motor });

// Inertial Sensor
pros::Imu imu_sensor(IMU_PORT);

// Pneumatic Piston
pros::ADIDigitalOut piston ('A');

/******************************************************************************************************
 * 
 * This code was intended to be used with a shaft encoder, but we did not implement one into our robot.
 * 
 *****************************************************************************************************/ 
// Shaft Encoder
// pros::ADIEncoder encoder ('A', 'B', MOTOR_ENCODER_ROTATIONS);

// void move_encoder(double voltage, double diameter, double distance) {
// 	// Note: need to delay on start up or else it will not work
// 	encoder.reset();

// 	while((abs((int)encoder.get_value())) < distance / (diameter * pi)) {
// 		left_group.move_voltage(voltage);
// 		right_group.move_voltage(voltage);
// 	}
// 	left_group.brake();
// 	right_group.brake();
// }

/**
 * This function is used to move the robot a certain distance using the motor's built-in encoders.
 * 
 * @param voltage: The voltage that the motors will be set to.
 * @param diameter: The diameter of the wheel.
 * @param distance: The distance that the robot will move.
 * @param forward: If true, the robot will move forward; if false, it will move backward.
 */
void move(int voltage, double diameter, double distance, bool forward) {
    // need to delay on start up
    double left_middle = left_middle_motor.get_position();
    double right_middle = right_middle_motor.get_position();
    double average = 0;

    int directionFactor = forward ? 1 : -1; // 1 for forward, -1 for backward

    while (abs(average) < distance / (pi * diameter)) {
        double lm_dif = left_middle_motor.get_position() - left_middle;
        double rm_dif = right_middle_motor.get_position() - right_middle;
        average = (lm_dif + rm_dif) / 2;

        left_group.move_voltage(voltage * offset * directionFactor);
        right_group.move_voltage(voltage * directionFactor);
    }

    left_group.brake();
    right_group.brake();
}


/**
 * This function is used to turn the robot a certain amount of degrees using the inertial sensor.
 * 
 * @param voltage: The voltage that the motors will be set to.
 * @param rotation: The amount of degrees that the robot will turn.
 */
void turn_imu(int voltage, int rotation) {
    int initialIntertialRotation = (int) imu_sensor.get_rotation();

	rotation *= 0.84;

	if (rotation > 0) {
		while ((int) imu_sensor.get_rotation() - initialIntertialRotation < rotation) {
			right_group.move_voltage(-voltage);
			left_group.move_voltage(voltage);
		}
	} else {
		while ((int) imu_sensor.get_rotation() - initialIntertialRotation > rotation) {
			right_group.move_voltage(voltage);
			left_group.move_voltage(-voltage);
		}
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
void turn(int voltage, int rotation) {
	// need to delay on start up
	double left_middle = left_middle_motor.get_position();
	double right_middle = right_middle_motor.get_position();
	double average = 0;

	if (rotation < 0) {
		voltage = -voltage;
		rotation = -rotation;
	}

	//double distance = (rotation * pi * ((length * length) + (width * width))) / (720 * length);
	double distance = (rotation * pi * length) / 360;

	while(fabs(average) < (distance / (pi * diameter))) {
		double lm_dif = left_middle_motor.get_position() - left_middle;
		double rm_dif = right_middle_motor.get_position() - right_middle;
		average = (fabs(lm_dif) + fabs(rm_dif)) / 2;
		
		left_group.move_voltage(voltage);
		right_group.move_voltage(-voltage);
	}
	left_group.brake();
	right_group.brake();
}

/**
 * @brief Linearly interpolates between two values.
 *
 * This function performs linear interpolation (lerp) between an initial value
 * and a target value based on a specified magnitude.
 *
 * @param initial_value The starting value.
 * @param target_value The target value to interpolate towards.
 * @param magnitude The magnitude of interpolation, typically in the range [0, 1].
 * @return The interpolated value as a 32-bit signed integer.
 */
std::int32_t lerp(std::uint32_t initial_value, std::int32_t target_value, double magnitude) {
	return static_cast<std::int32_t>((target_value - initial_value) * magnitude) + initial_value;
}

/**
 * @brief Linearly interpolates motor group voltage to a target value.
 *
 * Performs linear interpolation (lerp) of the voltage for a given motor group
 * towards a specified target voltage using a set interpolation magnitude.
 *
 * @param motor_group A reference to the `pros::Motor_Group` object to control.
 * @param target_voltage The desired target voltage for the motor group.
 */
void interpolate_motor_voltage(pros::Motor_Group& motor_group, std::int32_t target_voltage) {
	std::vector<std::uint32_t> voltages = motor_group.get_voltages();

	if (abs(voltages[0] - target_voltage) < INTERPOLATION_ERROR) {
		motor_group.move_voltage(target_voltage);
	}
	else {
		motor_group.move_voltage(lerp(voltages[0], target_voltage, INTERPOLATION_MAGNITUDE));
	}
}


void initialize() {
	pros::lcd::initialize();
	pros::ADIDigitalOut piston ('A', false);
	left_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	right_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	/**
	 * Dimensions Note: 
	 * the robot's dimensions are about from center of wheel to center of wheel 10.5 x 10 and 3/8 inches.
	 * Real dimensions are 14.5 x 13 inches.
	 * 
	 * TODO: Map out and write the autonomous instructions. Below is work in progress and subject to change.
	 * The autonomous instructions should fufill the following:
	 * 1. Move forward 44 inches
	 * 2. Pivot 90 degrees left
	 * 3. Move forward 47 inches
	 * 4. Pivot 90 degrees right
	 * 5. Dispense the pre-loaded triball into goal
	 * 6. Delay until next triball is caught from other robots catapult throw
	 * 7. Dispense the triball into goal
	 * Repeat steps 6 and 7 until all triballs are dispensed
	 * End autonomous
	 */
	// move(6000, 38.5);
	// turn_imu(3000, -90);
	// move(6000, 44);
	// turn_imu(3000, 90);
	// move(6000, 2);

	move(5000, diameter, 36.25, true);
	turn_imu(3000, -85);
	move(5000, diameter, 29, true);
	move(2000, diameter, 8, true);
	turn_imu(3000, 95);
	move(4000, diameter, 5, true);
	// move(6000, diameter, 2, true);
}

void opcontrol() {
	// Tank Drive
	/*
	while (true) {
		// Joystick input
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		// Calculate motor voltages based on joystick input
		int voltage_left = left * MAX_VOLTAGE / ANALOG_MAX_VALUE;
		int voltage_right = right * MAX_VOLTAGE / ANALOG_MAX_VALUE;

		// Apply interpolated motor voltages
		left_group.move_voltage(voltage_left);
		right_group.move_voltage(voltage_right);

		pros::delay(20); // Delay for loop iteration
	}
	*/
	
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
			pros::lcd::print(0, "%d %d %d", 0, 0, 0); // LCD display
		}
		else {
			// Calculate magnitude based on normalized x and y
			double normalized_x = static_cast<double>(x) / 127.0;
			double normalized_y = static_cast<double>(y) / 127.0;
			double magnitude = sqrt(normalized_x * normalized_x + normalized_y * normalized_y);

			// Calculate motor voltages based on joystick input
			double angle = atan2(y, x);
			double voltage_x = cos(angle) * MAX_VOLTAGE * magnitude;
			double voltage_y = sin(angle) * MAX_VOLTAGE * magnitude;

			// Distribute voltages for forward/backward and turning
			int voltage_left = voltage_y + voltage_x;
			int voltage_right = voltage_y - voltage_x;

			// Apply interpolated motor voltages
			// interpolate_motor_voltage(left_group, voltage_left);
			// interpolate_motor_voltage(right_group, voltage_right);
			left_group.move_voltage(voltage_left);
			right_group.move_voltage(voltage_right);

			// LCD display for debugging
			pros::lcd::print(0, "%d %d %d", static_cast<int>(angle), static_cast<int>(voltage_x), static_cast<int>(voltage_y));
		}

		// Intake control
		if (master.get_digital(DIGITAL_R1)) { // Intake
			intake_group.move_voltage(MAX_VOLTAGE);
		}
		else if (master.get_digital(DIGITAL_L1)) { // Release
			intake_group.move_voltage(-MAX_VOLTAGE);
		}
		else {
			intake_group.move_voltage(0);
		}

		// Piston control
		if (master.get_digital(DIGITAL_L2)) {
			piston.set_value(true);
		}
		else if (master.get_digital(DIGITAL_R2)){
			piston.set_value(false);
		}

		pros::delay(20); // Delay for loop iteration
	}
}
