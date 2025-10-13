#include "main.h"
#include "robot.h"
#include "auton.h"
#include "pid.h"
#include "lemlib/api.hpp"

using namespace pros;
using namespace std;
/**
 * A callback function for LLEMU's center button.
 *
 * [p]
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
	pros::lcd::set_text(1, "would you snowball 2 skibatons?");

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

int atn = 1;
int pressed = 1;
string autstr;

void competition_initialize() {
	while(true){
		if(Autonselect.get_value() == true){
			pressed ++;
		} else {
			pressed = 0;
		}
		if (pressed == 1){
			atn++;
		}

		if (atn == 0) {
			autstr = "NONE";
			con.print(0,0, "Aut 0: %s        ", autstr);
		}
		else if (atn == 1) {
			autstr = " RED RIGHT";
			con.print(0,0, "Aut 0: %s        ", autstr);
		}
		else if (atn == 2) {
			autstr = " RED LEFT";
			con.print(0,0, "Aut 0: %s        ", autstr);
		}
		else if (atn == 3) {
			autstr = " BLUE RIGHT";
			con.print(0,0, "Aut 0: %s        ", autstr);
		}
		else if (atn == 4) {
			autstr = " BLUE LEFT";
			con.print(0,0, "Aut 0: %s        ", autstr);
		}
		else if (atn == 5) {
			autstr = " SKILLS ";
			con.print(0,0, "Aut 0: %s        ", autstr);
		}
		else if (atn == 6) {
			autstr = " N/A ";
			con.print(0,0, "Aut 0: %s        ", autstr);
		}
		else if (atn == 7) {
			atn = 0;
		}
	}
}

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
bool scrapperToggle = false;
bool descoreToggle = false;
bool descore2Toggle = false;
bool dp1Toggle = false;
bool dp2Toggle = false;
void opcontrol() {
	
	int macro = 0;
	bool macroControl = false;
	int time = 0;

	while (true) {

		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)){
			atn--;
		} else if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){
			atn++;
		}

		int power = con.get_analog(ANALOG_LEFT_Y);
		int RX = con.get_analog(ANALOG_RIGHT_X);


		descore.set_value(descoreToggle);
		descore2.set_value(descore2Toggle);

		scrapper.set_value(scrapperToggle);

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
			intakes.move(-127);
		}
		else if (con.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intakes.move(127);
		}
		else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)){
			INTAKE.move(-127);
			INTAKE1.move(-127);
			INTAKE2.move(127);
		}
		else {
			intakes.move(0);
		}

		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
    	scrapperToggle = !scrapperToggle;   // flip the boolean
    	scrapper.set_value(scrapperToggle); // set piston to match
		}
		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)){
			descoreToggle = !descoreToggle;
			descore.set_value(descoreToggle);
			descore2Toggle = !descoreToggle;
			descore2.set_value(descore2Toggle);
		}
		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
			dp1Toggle = !dp1Toggle;
			dp1.set_value(dp1Toggle);
			dp2Toggle = !dp2Toggle;
			dp2.set_value(dp2Toggle);
		}
		// if(macroControl){
		// 	setConstants(0.03, 0 ,0);
		// 	if(macro == 0){
		// 		LDB.move(-calcPID(4200, roto.get_angle(), 0, 0));
		// 	} else if(macro == 1){
		// 		LDB.move(-calcPID(5800, roto.get_angle(), 0, 0));
		// 	} else if(macro == 2){
		// 		LDB.move(-calcPID(10000, roto.get_angle(), 0, 0));
		// 	}
		// }
		double chasstempC = ((RF.get_temperature() + RB.get_temperature() + LF.get_temperature() + LB.get_temperature()) /4);
		        if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0) {
            con.print(0, 0, "AUTON: %s      ", autstr);
        } else if (time % 100 == 0 && time % 150 != 0){
            con.print(1, 0, "IMU:               ", float(imu.get_heading()));
        } else if (time % 150 == 0){
            con.print(2,0, "C:%i  IN:%i  I:%i            ", int(chasstempC), int(INTAKE.get_temperature()), int(0));
        }

        time += 10;
        delay(10);
 	}
}



/* temp
25-35 - normal
45 - slowed
50 - 75% power
55 - 50% power
60 - cooked
65 - dead
80 - on fire
100 - boiling
*/
