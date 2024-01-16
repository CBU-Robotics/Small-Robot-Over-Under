#include "main.h"

const int LEFT_TOP_MOTOR_PORT = 11;
const int LEFT_Middle_MOTOR_PORT = 15;
const int LEFT_BOTTOM_MOTOR_PORT = 20;
const int RIGHT_TOP_MOTOR_PORT = 2;
const int RIGHT_Middle_MOTOR_PORT = 5;
const int RIGHT_BOTTOM_MOTOR_PORT = 10;

const int INTERTIAL_SENSOR_PORT = 4;
const int VEX_MAX_VOLTAGE = 12000; // Note: If motors have r10 board do not go above 10,000mV and switch directions.
const int MAX_VOLTAGE = VEX_MAX_VOLTAGE - 0;
const int ANALOG_MAX_VALUE = 127;
const double INTERPOLATION_MAGNITUDE = 0.01;
const int INTERPOLATION_ERROR = 30;
const double pi = 3.14159265358979323846;
const int IMU_PORT = 14;

//The amount the left side needs to catch up to the right
const double offset = 1.06;

// Controller and motor setup
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_top_motor(LEFT_TOP_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, MOTOR_ENCODER_ROTATIONS);
pros::Motor left_middle_motor(LEFT_Middle_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, MOTOR_ENCODER_ROTATIONS);
pros::Motor left_bottom_motor(LEFT_BOTTOM_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, MOTOR_ENCODER_ROTATIONS);
pros::Motor right_top_motor(RIGHT_TOP_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, MOTOR_ENCODER_ROTATIONS);
pros::Motor right_middle_motor(RIGHT_Middle_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, MOTOR_ENCODER_ROTATIONS);
pros::Motor right_bottom_motor(RIGHT_BOTTOM_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, MOTOR_ENCODER_ROTATIONS);

// Motor groups and brake modes
pros::Motor_Group left_group({ left_top_motor, left_middle_motor, left_bottom_motor });
pros::Motor_Group right_group({ right_top_motor, right_middle_motor, right_bottom_motor });

// Inertial Sensor
pros::Imu imu_sensor(14);


/******************************************************************************************************
 * 
 * This code was intended to be used with a shaft encoder, but we did not implement one into our robot.
 * 
 *****************************************************************************************************/ 
// Shaft Encoder
pros::ADIEncoder encoder ('A', 'B', MOTOR_ENCODER_ROTATIONS);

void move_distance(double voltage, double diameter, double distance) {
	// Note: need to delay on start up or else it will not work
	encoder.reset();

	while((abs((int)encoder.get_value())) < distance / (diameter * pi)) {
		left_group.move_voltage(voltage);
		right_group.move_voltage(voltage);
	}
	left_group.brake();
	right_group.brake();
}

/*
 * This function is used to move the robot a certain distance using the motor's built in encoders.
 * 
 * @param voltage: The voltage that the motors will be set to.
 * @param diameter: The diameter of the wheel.
 * @param distance: The distance that the robot will move.
 */
void move_distance_motor(int voltage, double diameter, double distance) {
	// Note: need to delay on start up or else it will not work
	int left_top = left_top_motor.get_position();
	int left_middle = left_middle_motor.get_position();
	int left_bottom = left_bottom_motor.get_position();
	int right_top = right_top_motor.get_position();
	int right_middle = right_middle_motor.get_position();
	int right_bottom = right_bottom_motor.get_position();
	int average = 0;

	while(abs(average) < distance / (pi * diameter)) {
		int lt_dif = left_top_motor.get_position() - left_top;
		int lm_dif = left_middle_motor.get_position() - left_middle;
		int lb_dif = left_bottom_motor.get_position() - left_bottom;
		int rt_dif = right_top_motor.get_position() - right_top;
		int rm_dif = right_middle_motor.get_position() - right_middle;
		int rb_dif = right_bottom_motor.get_position() - right_bottom;
		average = (lt_dif + lm_dif + lb_dif + rt_dif + rm_dif + rb_dif) / 6;
		
		left_group.move_voltage(voltage * offset);
		right_group.move_voltage(voltage);
	}
	left_group.brake();
	right_group.brake();
}

bool autoTurn(int degrees, int voltage) {
    int degreesTurned = 0;
    int initialIntertialRotation = (int) imu_sensor.get_rotation();
  	while (!((degreesTurned < degrees + 3) && (degreesTurned > degrees - 3))) {
      pros::lcd::print(2, "Degrees turned: %d\n", degreesTurned);
      degreesTurned = abs(initialIntertialRotation - (int) imu_sensor.get_rotation());
    if (voltage > 0) {
        left_group.move_voltage(voltage);
    } else {
        right_group.move_voltage(-voltage);
    }
    pros::lcd::print(3, "IMU get rotation: %d degrees\n", (int) imu_sensor.get_rotation());
		pros::delay(20);
	}
  left_group.brake();
  right_group.brake();
  return false;
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
	left_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	right_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
}

void disabled() {}

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
	/**
	 * Dimensions Note: 
	 * the robot's dimensions are about from center of wheel to center of wheel 10.5 by 10 and 3/8 inches.
	 * Real dimensions are TODO: measure
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
	**/
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

		pros::delay(20); // Delay for loop iteration
	}
}
