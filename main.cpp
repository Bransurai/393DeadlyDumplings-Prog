#include "main.h"
#include "auton.h"
#include "robot.h"
#include "pid.h"

// #include "robot.cpp"

using namespace pros;
using namespace std;
///#include "robot.cpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	// static bool pressed = false;
	// pressed = !pressed;
	// if (pressed) {
	// 	pros::lcd::set_text(2, "Illegals in my yard Illegals in my yard Illegals in my yard Throw them some pesos, and they work so hard Illegals in my yard Illegals in my yard Illegals in my yard I don't even ask if they got green card");
	// } else {
	// 	pros::lcd::clear_line(2);
	// }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Deadly Dumplings");

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

void autonomous() {
auton2();
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
	
	bool arcade = true;
bool catapressed;
bool piston = true; 
bool ipistontoggle = false;
bool intakepistontoggle = false;
		int time = 0;
int cycle = 0;
while(true){

// Sensors

typedef struct __attribute__((__packed__)) optical_rgb_s {
  double red;
  double green;
  double blue;
  double brightness;
} optical_raw_s_t;

	//distance
		cycle++;
            // if ((cycle+1) % 3 == 0) con.print(0, 0, "Error | %2f", target-position); 
            if ((cycle+2) % 3 == 0) con.print(1, 0, "Integral | %2f", float(imu.get_heading())); //autstr //%s
            // if ((cycle+3) % 3 == 0) con.print(2, 0, "Integral: %f", viewIntegral);

	
//colorsorter

if(optical.get_hue() > 7 && optical.get_hue() < 30){
	Intake1.move(-127); //-127
	Intake2.move(127);
	Intake3.move(127);
	delay(1000);
	Intake1.move(127);
	Intake2.move(-127);
	Intake3.move(127);
} else{

	//Intake Stuff

// if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ||
//     con.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
//   set_intake(127);         // INTAKE
// } else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ||
//            con.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
//   set_intake(-127);        // OUTTAKE
// } else {
//   set_intake(0);
// }

	//for the PTO true = in and false = out
//Intake to Basket

if(con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){ //PTO
	piston=!piston;
	basket.set_value(piston);
}
if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { //Outake
	basket.set_value(true);
	Intake1.move(-127);
	Intake2.move(127);
	Intake3.move(-127);
} else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { //Intake
	basket.set_value(true);
	Intake1.move(127);
	Intake2.move(-127);
	Intake3.move(127);
} else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){ //low goal
	basket.set_value(false);
	Intake1.move(-127); //-127
	Intake2.move(127);
	Intake3.move(127);
} else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ //high goal
	basket.set_value(false);
	Intake1.move(-127);
	Intake2.move(-127);
	Intake3.move(127);
}else {
	Intake1.move(0);
	Intake2.move(0);
	Intake3.move(0);
}

}

if(con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){ 
	driveTurn2(90, 1000);
}

//Drivecontrol

if (arcade){
//Arcade Drive
int power = con.get_analog(ANALOG_LEFT_Y);
int RX = con.get_analog(ANALOG_RIGHT_X);

int turn = int(abs(RX) * RX / 75);
int left = power + turn;
int right = power - turn;

LF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
LB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
LM1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
RF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
RB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
RM1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);


LF.move(left);
LB.move(left);
LM1.move(left);
RF.move(right);
RB.move(right);
RM1.move(right);
}
time += 1;
cycle++;
}

}
