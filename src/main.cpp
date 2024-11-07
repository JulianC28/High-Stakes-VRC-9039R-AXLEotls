#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/lvgl.h"
#include "auton.hpp"

// LV_IMG_DECLARE(axolotlFire);

// variables
bool setupDone, setupDrive, setupColor, setupSide = false;
int pistonValue = 1;
int wingValue = 0;
bool jake, red, rings;

// motor setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup rightDrive({1, 2}); // creates the right drivetrain motor group with forwards ports 1 & 2
pros::MotorGroup leftDrive({-3, -4}); // creates the left drivetrain motor group with backwards ports 3 & 4
pros::MotorGroup lift({7, 8}); // creates the lift motor group with forwards ports 7 & 8
pros::Motor intake(-5); // creates the intake motor with backwards port 5
pros::Motor conveyor(6); // creates the conveyor motor with forwards port 6
pros::Motor wing(9); // creates the wing motor with forward port 8

// adi setup
pros::adi::DigitalOut piston ('A');
pros::ADIEncoder verticalEncoder ('C', 'D', true);

// odom setup
pros::Imu inertialSensor(7);
lemlib::TrackingWheel verticalWheel(&verticalEncoder, lemlib::Omniwheel::OLD_275, -1.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
                              &rightDrive, // right motor group
                              13.5, // Track Width Not Found Yet, // 13.5 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              428.5714286, // drivetrain rpm is ~429 rpm
                              2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(&verticalWheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to null
                            nullptr, // horizontal tracking wheel 1, set to null
                            nullptr, // horizontal tracking wheel 2, set to null
                            &inertialSensor // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(25, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              200, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              55, // derivative gain (kD)
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
    pros::lcd::initialize;
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

void competition_initialize(void) {
    
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
    if(red == true && rings == true){
        intake.set_zero_position(0);
        chassis.setPose(0, 0, 0); // set position to x:0, y:0, heading:0
        chassis.moveToPoint(0, 10, 5000);
    }
    if(red == true && rings == false){
        intake.set_zero_position(0);
        chassis.setPose(0, 0, 0); // set position to x:0, y:0, heading:0
        chassis.moveToPoint(0, 10, 5000);
    }
    if(red == false && rings == true){
        intake.set_zero_position(0);
        chassis.setPose(0, 0, 0); // set position to x:0, y:0, heading:0
        chassis.moveToPoint(0, 10, 5000);
    }
    if(red == false && rings == false){
        intake.set_zero_position(0);
        chassis.setPose(0, 0, 0); // set position to x:0, y:0, heading:0
        chassis.moveToPoint(0, 10, 5000);
    }
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
    /*
    lv_obj_t * img1 = lv_img_create(lv_scr_act());
    lv_img_set_src(img1, &axolotlFire);
    lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);  
    */
   
    // drive control setup
    while(setupDrive == false) {
        pros::screen::print(TEXT_LARGE, 1, "jake drive = A"); // prints to brain
        pros::screen::print(TEXT_LARGE, 2, "liam drive = B"); // prints to brain
        if(controller.get_digital(DIGITAL_A)){
            jake = true; // sets our drive with the clamp as the front
            setupDrive = true; // finishes the first setup
        }
        else if(controller.get_digital(DIGITAL_B)){
            jake = false; // sets our drive with the intake as the front
            setupDrive = true; // finishes the first setup
        }
		pros::delay(20); // delay to save resources
    }

    while(setupDrive == true && setupColor == false) {
        pros::screen::erase(); // erases brain
        pros::screen::print(TEXT_LARGE, 1, "Red = X"); // prints to brain
        pros::screen::print(TEXT_LARGE, 2, "Blue = Y"); // prints to brain
        if(controller.get_digital(DIGITAL_X)){
            red = true; // sets our auton to red alliance
            setupColor = true; // finishes the second setup
        }
        else if(controller.get_digital(DIGITAL_Y)){
            red = false; // sets our auton to blue alliance
            setupColor = true; // finishes the second setup
        }
        pros::delay(20); // delay to save resources
    }

    while(setupDrive == true && setupColor == true && setupSide == false) {
        pros::screen::erase(); // erases brain
        pros::screen::print(TEXT_LARGE, 1, "Rings = L1 "); // prints to brain
        pros::screen::print(TEXT_LARGE, 2, "Mobile Goal = R1"); // prints to brain
        if(controller.get_digital(DIGITAL_L1)){
            rings = true; // sets our auton to the side with 4 rings
            setupSide = true; // finishes the third setup
        }
        else if(controller.get_digital(DIGITAL_R1)){
            rings = false; // sets our auton to the side with an extra mobile goal
            setupSide = true; // finishes the third setup
        }
        pros::delay(20); // delay to save resources
    }

    while (setupDrive == true && setupColor == true && setupSide == true) {
        pros::screen::erase(); // erases brain

        // tank drive control scheme
        if(jake == true){
        int tankLeft = controller.get_analog(ANALOG_LEFT_Y); // the left analog stick controls the left side of the drive train
        int tankRight = controller.get_analog(ANALOG_RIGHT_Y); // the right analog stick controls the left side of the drive train         
        chassis.tank(tankLeft, tankRight); // creates a tank drive scheme
        }
        else if(jake == false){
            int tankLeft = -controller.get_analog(ANALOG_RIGHT_Y); // the right analog stick controls the left side of the drive train     
            int tankRight = -controller.get_analog(ANALOG_LEFT_Y); // the left analog stick controls the left side of the drive train
            chassis.tank(tankLeft, tankRight); // creates a tank drive scheme
        }

        // intake + conveyor control scheme
        if(controller.get_digital(DIGITAL_R2)) { 
            intake.move(127); // moves the intake forwards at max speed
			conveyor.move(127); // moves the conveyor forwards at max speed
        }
        else if(controller.get_digital(DIGITAL_L2)) {
            intake.move(-127); // moves the intake forwards at max speed
			conveyor.move(-127); // moves the conveyor backwards at max speed
        }
        else {
            intake.move(0); // stops the intake from moving
			conveyor.move(0); // stops the conveyor from moving
        }
       
        // clamp control scheme
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
       
		// lift control scheme
		if(controller.get_digital(DIGITAL_R1)){
			lift.move(95); // moves the lift forwards at ~75% velocity
		}
		else if(controller.get_digital(DIGITAL_L1)){
			lift.move(-95); // moves the lift backwards at ~75% velocity
		}
		else{
			lift.move(0); // stops the lift from moving
		}

        // wing control scheme
        if(controller.get_digital(DIGITAL_Y) && wingValue == 0) {
            wing.move_absolute(-240, 100);
            pros::delay(500);
            wingValue = 1;
        }
        else if(controller.get_digital(DIGITAL_Y) && wingValue == 1) {
            wing.move_absolute(0, 100);
            pros::delay(500);
            wingValue = 0;
        }

        pros::delay(20); // delay to save resources
    }
}

