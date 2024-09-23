#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// variables
int driveChoicer = 0;
int autonChoicer = 0;

// motor setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftDrive({-1, -2});    // creates a motor group with negative ports 1 & 2
pros::MotorGroup rightDrive({3, 4});  // creates a motor group with forwards ports 3 & 4
pros::MotorGroup intake({-5, 6}); // creates a motor group with forwards port 6 & backwards port 5

// sensor setup
pros::Imu inertialSensor(7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
                              &rightDrive, // right motor group
                              0, // Track Width Not Found Yet, // __ inch track width
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
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
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
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
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
	while (true) {
		pros::lcd::clear_line(1);
		pros::lcd::clear_line(2);

		pros::lcd::print(1, "Cycle Through Autons with R1 & L1");
		pros::lcd::print(2, "Cycle Through Drives with R2 & L2");

		switch(autonChoicer) {
			case 0:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);
			
			pros::lcd::print(4, "Previous: Nothing Selected");
			pros::lcd::print(5, "Current: No Auton");
			pros::lcd::print(6, "Next: Prototype");
			break;

			case 1:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);

			pros::lcd::print(4, "Previous: No Auton");
			pros::lcd::print(5, "Current: Prototype");
			pros::lcd::print(6, "Next: AWP, Red Alliance, Rings");
			break;

			case 2:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);
			
			pros::lcd::print(4, "Previous: Prototype");
			pros::lcd::print(5, "Current: AWP, Red Alliance, Rings");
			pros::lcd::print(6, "Next: AWP, Red Alliance, Mobile Goal");
			break;

			case 3:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);

			pros::lcd::print(4, "Previous: AWP, Red Alliance, Rings");
			pros::lcd::print(5, "Current: AWP, Red Alliance, Mobile Goal");
			pros::lcd::print(6, "Next: AWP, Blue Alliance, Rings");
			break;

			case 4:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);

			pros::lcd::print(4, "Previous: AWP, Red Alliance, Mobile Goal");
			pros::lcd::print(5, "Current: AWP, Blue Alliance, Rings");
			pros::lcd::print(6, "Next: AWP, Blue Alliance, Mobile Goal");
			break;

			case 5:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);

			pros::lcd::print(4, "Previous: AWP, Blue Allianve, Rings");
			pros::lcd::print(5, "Current: AWP, Blue Alliance, Mobile Goal");
			pros::lcd::print(6, "Next: No AWP, Red Alliance, Rings");
			break;

			case 6:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);

			pros::lcd::print(4, "AWP, Blue Alliance, Mobile Goal");
			pros::lcd::print(5, "Current: No AWP, Red Alliance, Rings");
			pros::lcd::print(6, "Next: No AWP, Red Alliance, Mobile Goal");
			break;

			case 7:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);

			pros::lcd::print(4, "Previous: No AWP, Red Alliance, Rings");
			pros::lcd::print(5, "Current: No AWP, Red Alliance, Mobile Goal");
			pros::lcd::print(6, "Next: No AWP, Blue Alliance, Rings");
			break;

			case 8:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);

			pros::lcd::print(4, "Previous: No AWP, Red Alliance, Mobile Goal");
			pros::lcd::print(5, "Current: No AWP, Blue Alliance, Rings");
			pros::lcd::print(6, "Next: No AWP, Blue Alliance, Mobile Goal");
			break;

			case 9:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);

			pros::lcd::print(4, "Previous: No AWP, Blue Alliance, Rings");
			pros::lcd::print(5, "Current: No AWP, Blue Alliance, Mobile Goal");
			pros::lcd::print(6, "Nothing Selected");
			break;

			default:
			pros::lcd::clear_line(4);
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);

			pros::lcd::print(4, "Lost in the sauce? Press A to reset");
		}
		switch(driveChoicer) {
			case 0:
			pros::lcd::clear_line(7);
			pros::lcd::clear_line(8);
			pros::lcd::clear_line(9);

			pros::lcd::print(7, "Previous: Nothing Selected");
			pros::lcd::print(8, "Current Drive Method: Tank");
			pros::lcd::print(9, "Next: Curvature Drive");
			break;

			case 1:
			pros::lcd::clear_line(7);
			pros::lcd::clear_line(8);
			pros::lcd::clear_line(9);

			pros::lcd::print(7, "Previous: Tank Drive");
			pros::lcd::print(8, "Drive Method: Curvature");
			pros::lcd::print(9, "Next: Arcade Drive");
			break;

			case 2:
			pros::lcd::clear_line(7);
			pros::lcd::clear_line(8);
			pros::lcd::clear_line(9);

			pros::lcd::print(7, "Previous: Curvature Drive");
			pros::lcd::print(8, "Drive Method: Arcade");
			pros::lcd::print(9, "Next: Nothing Selected");
			break;

			default:
			pros::lcd::clear_line(7);
			pros::lcd::clear_line(8);
			pros::lcd::clear_line(9);

			pros::lcd::print(7, "Lost in the sauce? Press A to reset");
		}

		if(controller.get_digital(DIGITAL_R1)) {
			autonChoicer++;
		}
		else if(controller.get_digital(DIGITAL_L1)) {
			autonChoicer--;
		}
		else if(controller.get_digital(DIGITAL_R2)) {
			driveChoicer++;
		}
		else if(controller.get_digital(DIGITAL_L2)) {
			driveChoicer--;
		}
		else if(controller.get_digital(DIGITAL_A)) {
			autonChoicer = 0;
			driveChoicer = 0;
		}

		pros::delay(25);
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
    chassis.turnToHeading(90, 100000);
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

	while (true) { // infinite loop

		// DRIVE CONTROLS

		// tank drive control scheme
		if (driveChoicer == 0) {
			int tankLeft = leftDrive.move(controller.get_analog(ANALOG_LEFT_Y)*0.87);               
			int tankRight = rightDrive.move(controller.get_analog(ANALOG_RIGHT_Y)*0.87);                   
			chassis.tank(tankLeft, tankRight);
		}

		// curvature drive control scheme
		if (driveChoicer == 1) {
			int curvatureLeft = leftDrive.move(controller.get_analog(ANALOG_LEFT_Y)*0.87);                    
			int curvatureRight = rightDrive.move(controller.get_analog(ANALOG_RIGHT_X)*0.87);                    
			chassis.curvature(curvatureLeft, curvatureRight);
		}

		// arcade drive control scheme
		if (driveChoicer == 2) {
			int arcadeLeft = leftDrive.move(controller.get_analog(ANALOG_LEFT_Y)*0.87);                      
			int arcadeRight = rightDrive.move(controller.get_analog(ANALOG_RIGHT_X)*0.87);
			chassis.arcade(arcadeLeft, arcadeRight);
		}

		// intake control scheme
		
		if(controller.get_digital(DIGITAL_R2)) {
			intake.move_velocity(200);
		}
		else if(controller.get_digital(DIGITAL_L2)) {
			intake.move_velocity(-200);
		}
		else {
			intake.move_velocity(0);
		}

		// clamp control scheme


		pros::delay(25);
	}
}
