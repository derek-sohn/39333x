#include "main.h"
#include "robot.h"
#include "auton.h"
#include "pid.h"

using namespace pros;
using namespace std;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
	
	int macro = 0;
	bool macroControl = false;

	while (true) {
		int power = con.get_analog(ANALOG_LEFT_Y);
		int RX = con.get_analog(ANALOG_RIGHT_X);


		int turn = int(RX);
		//int turn = int(abs(RX) * RX / 127);
		//int turn = int(pow(RX, 3) / pow(127, 2));
		int left = power + turn;
		int right = power - turn;

		LF.move(left);
		LM.move(left);
		LB.move(left);
		RF.move(right);
		RM.move(right);
		RB.move(right);

		if (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			INTAKE.move(127);
		}
		else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			INTAKE.move(-127);
		}
		else {
			INTAKE.move(0);
		}
		if(con.get_digital(E_CONTROLLER_DIGITAL_L1)){
			LDB.move(127);
			macroControl = false;
		} else if(con.get_digital(E_CONTROLLER_DIGITAL_L2)){
			LDB.move(-127);
			macroControl = false;
		} else if(macroControl == false){
			LDB.move(0);
		}
		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			macro ++;
			macroControl = true;
		}
		if(macroControl){
			setConstants(0.03, 0 ,0);
			if(macro == 0){
				LDB.move(-calcPID(4200, roto.get_angle(), 0, 0));
			} else if(macro == 1){
				LDB.move(-calcPID(5800, roto.get_angle(), 0, 0));
			} else if(macro == 2){
				LDB.move(-calcPID(10000, roto.get_angle(), 0, 0));
			}
		}
	}
}


