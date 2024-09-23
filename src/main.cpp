#include "main.h"
#include "colors.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// variables
bool driveDone1, driveDone2, autonDone1, autonDone2, autonDone3, autonDone4, autonSkip = false;
bool autonRed, autonExtra, autonRings, autonAWP, noAuton, driveTank, driveCurve, driveArcade;
int pistonValue = 1;
int red = 16711680;
int blue = 65535;
int yellow = 16776960;
int black = 0;
int white = 16777215;
std::string autonColorString, autonRingsString, autonAWPString, autonSkipString, driveString;

// motor setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup rightDrive({1, 2});    // creates a motor group with negative ports 1 & 2
pros::MotorGroup leftDrive({-3, -4});  // creates a motor group with forwards ports 3 & 4
pros::MotorGroup intake({-5, 6}); // creates a motor group with forwards port 6 & backwards port 5

// adi setup
pros::ADIDigitalOut piston ('A');

// sensor setup
pros::Imu inertialSensor(7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
                              &rightDrive, // right motor group
                              13.5, // Track Width Not Found Yet, // 13.5 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              372.857142882, // drivetrain rpm is ~373 rpm
                              2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to null
                            nullptr, // horizontal tracking wheel 1, set to null
                            nullptr, // horizontal tracking wheel 2, set to null
                            &inertialSensor // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              2, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              6, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              100 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              2, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
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

void competition_initialize() {
	pros::screen_touch_status_s_t status = pros::screen::touch_status();
	while(autonDone1 == false) {
		pros::screen::erase();
		pros::screen::set_pen(red);
		pros::screen::draw_rect(0, 0, 160, 200);
		pros::screen::set_pen(blue);
		pros::screen::draw_rect(160, 200, 320, 0);
		pros::screen::set_pen(yellow);
		pros::screen::draw_rect(320, 0, 480, 200);
		pros::screen::set_pen(black);
		pros::screen::print(TEXT_LARGE, 80, 100, "RED");
		pros::screen::print(TEXT_LARGE, 240, 100, "BLUE");
		pros::screen::print(TEXT_LARGE, 400, 100, "EXTRA");
		if(status.x <= 150){
			autonRed, autonDone1 = true;
			autonColorString = "Red Alliance";
		}
		else if(170 <= status.x <= 310) {
			autonRed = false;
			autonDone1 = true;
			autonColorString = "Blue Alliance";
		}
		else if(330 <= status.x <= 470) {
			autonExtra, autonDone1 = true;
		}
	}
	while(autonSkip == false && autonDone1 == true && autonExtra == true) {
		pros::screen::erase();
		pros::screen::set_pen(red);
		pros::screen::draw_rect(0, 170, 240, 0);
		pros::screen::set_pen(yellow);
		pros::screen::draw_rect(240, 0, 480, 170);
		pros::screen::set_pen(black);
		pros::screen::draw_rect(0, 170, 480, 200);
		pros::screen::print(TEXT_LARGE, 120, 85, "NO AUTON");
		pros::screen::print(TEXT_LARGE, 360, 85, "PROTOTYPE");
		pros::screen::set_pen(white);
		pros::screen::print(TEXT_LARGE, 240, 185, "BACK");
		if(status.y >= 170) {
			autonDone1, autonExtra = false;
		}
		else if(status.x <= 230 && status.y <= 160) {
			noAuton, autonSkip = true;
			autonSkipString = "No Auton";
			autonColorString = "";
			autonRingsString = "";
			autonAWPString = "";
		}
		else if(250 <= status.x <= 470 && status.y <=160) {
			noAuton = false;
			autonSkip = true;
			autonSkipString = "Prototype";
			autonColorString = "";
			autonRingsString = "";
			autonAWPString = "";
		}
	}
	while(autonDone2 == false && autonDone1 == true && autonSkip == false) {
		pros::screen::erase();
		pros::screen::set_pen(red);
		pros::screen::draw_rect(0, 170, 240, 0);
		pros::screen::set_pen(yellow);
		pros::screen::draw_rect(240, 0, 480, 170);
		pros::screen::set_pen(black);
		pros::screen::draw_rect(0, 170, 480, 200);
		pros::screen::print(TEXT_LARGE, 120, 85, "RINGS");
		pros::screen::print(TEXT_LARGE, 360, 85, "GOAL");
		pros::screen::set_pen(white);
		pros::screen::print(TEXT_LARGE, 240, 185, "BACK");
		if(status.y >= 170) {
			autonDone1 = false;
		}
		else if(status.x <= 230 && status.y <= 160) {
			autonRings, autonDone2 = true;
			autonRingsString = "Extra Rings";
		}
		else if(250 <= status.x <= 470 && status.y <=160) {
			autonRings = false;
			autonDone2 = true;
			autonRingsString = "Extra Mobile Goal";
		}
	}
	while(autonDone3 == false && autonDone2 == true && autonDone1 == true && autonSkip == false) {
		pros::screen::erase();
		pros::screen::set_pen(red);
		pros::screen::draw_rect(0, 170, 240, 0);
		pros::screen::set_pen(blue);
		pros::screen::draw_rect(240, 0, 480, 170);
		pros::screen::set_pen(black);
		pros::screen::draw_rect(0, 170, 480, 200);
		pros::screen::print(TEXT_LARGE, 120, 85, "AWP");
		pros::screen::print(TEXT_LARGE, 360, 85, "NO AWP");
		pros::screen::set_pen(white);
		pros::screen::print(TEXT_LARGE, 240, 185, "BACK");
		if(status.y >= 170) {
			autonDone2 = false;
		}
		else if(status.x <= 230 && status.y <= 160) {
			autonAWP, autonDone3 = true;
			autonAWPString = "With AWP";
		}
		else if(250 <= status.x <= 470 && status.y <=160) {
			autonAWP = false;
			autonDone3 = true;
			autonAWPString = "Without AWP";
		}
	}
	while((autonDone4 == false && autonDone3 == true && autonDone2 == true && autonDone1 == true) || autonSkip == true) {
		pros::screen::erase();
		pros::screen::set_pen(yellow);
		pros::screen::draw_rect(0, 170, 240, 0);
		pros::screen::set_pen(red);
		pros::screen::draw_rect(240, 0, 480, 170);
		pros::screen::set_pen(black);
		pros::screen::draw_rect(0, 170, 480, 200);
		pros::screen::print(TEXT_LARGE, 120, 85, "CONFIRM");
		pros::screen::print(TEXT_LARGE, 360, 85, "GO BACK");
		pros::screen::set_pen(white);
		pros::screen::print(TEXT_MEDIUM, 2, "Current Auton: ", autonColorString, autonRingsString, autonAWPString, autonSkipString);
		if(status.x <= 230 && status.y <= 160) {
			autonDone4 = true;
		}
		else if(250 <= status.x <= 470 && status.y <=160) {
			autonDone4, autonSkip = false;
		}
	}
	while(autonDone4 == true && driveDone1 == false) {
		pros::screen::erase();
		pros::screen::set_pen(red);
		pros::screen::draw_rect(0, 0, 160, 200);
		pros::screen::set_pen(blue);
		pros::screen::draw_rect(160, 200, 320, 0);
		pros::screen::set_pen(yellow);
		pros::screen::draw_rect(320, 0, 480, 200);
		pros::screen::set_pen(black);
		pros::screen::print(TEXT_LARGE, 80, 100, "TANK");
		pros::screen::print(TEXT_LARGE, 240, 100, "CURVATURE");
		pros::screen::print(TEXT_LARGE, 400, 100, "ARCADE");
		if(status.x <= 150){
			driveTank, driveDone1 = true;
			driveString = "Tank Drive";
		}
		else if(170 <= status.x <= 310) {
			driveCurve, driveDone1 = true;
			driveString = "Curvature Drive";
		}
		else if(330 <= status.x <= 470) {
			driveArcade, driveDone1 = true;
			driveString = "Arcade Drive";
		}
	}
	while(driveDone2 == false && driveDone1 == true) {
		pros::screen::erase();
		pros::screen::set_pen(yellow);
		pros::screen::draw_rect(0, 170, 240, 0);
		pros::screen::set_pen(red);
		pros::screen::draw_rect(240, 0, 480, 170);
		pros::screen::set_pen(black);
		pros::screen::draw_rect(0, 170, 480, 200);
		pros::screen::print(TEXT_LARGE, 120, 85, "CONFIRM");
		pros::screen::print(TEXT_LARGE, 360, 85, "GO BACK");
		pros::screen::set_pen(white);
		pros::screen::print(TEXT_MEDIUM, 2, "Current Drive: ", driveString);
		if(status.x <= 230 && status.y <= 160) {
			driveDone2 = true;
		}
		else if(250 <= status.x <= 470 && status.y <=160) {
			driveDone2 = false;
			driveTank, driveCurve, driveArcade = false;
		}
	}
	while(autonDone4 == true && driveDone2 == true) {
	
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


void autonomous() {
		// set position to x:0, y:0, heading:0
    	chassis.setPose(0, 0, 0);
    	// turn to face heading 90 with a very long timeout
    	// chassis.turnToHeading(180, 100000);
		// move 48" forwards
    	// chassis.moveToPoint(0, 48, 10000);]
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

// tank drive control scheme
		if (driveTank == true) {
			int tankLeft = controller.get_analog(ANALOG_LEFT_Y);         
			int tankRight = controller.get_analog(ANALOG_RIGHT_Y);                   
			chassis.tank(tankLeft, tankRight);
		}

		// curvature drive control scheme
		if (driveCurve == true) {
			int curvatureLeft = controller.get_analog(ANALOG_LEFT_Y);                   
			int curvatureRight = controller.get_analog(ANALOG_RIGHT_X);                   
			chassis.curvature(curvatureLeft, curvatureRight);
		}

		// arcade drive control scheme
		if (driveArcade == true) {
			int arcadeLeft = controller.get_analog(ANALOG_LEFT_Y);;                     
			int arcadeRight = controller.get_analog(ANALOG_RIGHT_X);
			chassis.arcade(arcadeLeft, arcadeRight);
		}
		
		// Intake Control Scheme
		if(controller.get_digital(DIGITAL_R2)) {
			intake.move(127);
		}
		else if(controller.get_digital(DIGITAL_L2)) {
			intake.move(-127);
		}
		else {
			intake.move(0);
		}
		
		//clump

		if(controller.get_digital(DIGITAL_A) && pistonValue == 1){
			piston.set_value(false);
			pros::delay(500);
			pistonValue = 0;
		}
		else if(controller.get_digital(DIGITAL_A) && pistonValue == 0){
			piston.set_value(true);
			pros::delay(500);
			pistonValue = 1;
		}
		
		pros::delay(20);

	}
}