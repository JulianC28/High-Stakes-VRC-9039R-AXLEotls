#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/lvgl.h"


LV_IMG_DECLARE(axolotlFire);


// variables
bool setupDrive = false, setupColor = false, setupSide = false, test = false, justMove = false, skills = false, modded = false, regularAuton = false, red, rings, rowan;


// motor setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup rightDrive({1, 2}); // creates the right drivetrain motor group with forwards ports 1 & 2
pros::MotorGroup leftDrive({-3, -4}); // creates the left drivetrain motor group with backwards ports 3 & 4
pros::Motor intake(-5); // creates the intake motor with backwards port 5
pros::Motor conveyor(-6); // creates the conveyor motor with forwards port 6


// adi setup
pros::adi::DigitalOut piston ('A');
pros::adi::DigitalOut lift ('B');
pros::ADIEncoder verticalEncoder ('C', 'D', true);
pros::adi::DigitalOut wing ('E');


// odom setup
pros::Imu inertialSensor(7);
// lemlib::TrackingWheel verticalWheel(&verticalEncoder, lemlib::Omniwheel::OLD_275, -1.5);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
                              &rightDrive, // right motor group
                              13.5, // 13.5 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              428.5714286, // drivetrain rpm is ~429 rpm
                              2 // horizontal drift is 2 (for now)
);


// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null (temporary)
                            nullptr, // vertical tracking wheel 2, set to null
                            nullptr, // horizontal tracking wheel 1, set to null
                            nullptr, // horizontal tracking wheel 2, set to null
                            &inertialSensor // inertial sensor
);


// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
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
    lvgl_init;
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
    if(skills == true && regularAuton == false){
        intake.set_zero_position(0);
        conveyor.set_zero_position(0);
        chassis.setPose(0, 0, 0);
        conveyor.move_absolute(2000, 127);
        pros::delay(1000);
        chassis.moveToPoint(0, 14, 1500, {.maxSpeed = 100});
        chassis.turnToHeading(-90, 1500, {.maxSpeed = 100});
        chassis.moveToPoint(25, 14, 1500, {.forwards = false, .maxSpeed = 100});
        chassis.waitUntilDone();
        piston.set_value(true);
        chassis.turnToHeading(90, 2000, {.maxSpeed = 75});
        chassis.waitUntilDone();
        intake.move_absolute(15000, 200);
        conveyor.move_absolute(15000, 200);
        chassis.moveToPoint(60, 14, 5000, {.forwards = true, .maxSpeed = 50});
        chassis.moveToPoint(45, 14, 2000, {.forwards = false, .maxSpeed = 100});
        chassis.turnToHeading(180, 2000, {.maxSpeed = 100});
        chassis.waitUntilDone();
        intake.move_absolute(30000, 200);
        conveyor.move_absolute(30000, 200);
        chassis.moveToPoint(45, 5, 2000, {.forwards = true, .maxSpeed = 100});
        chassis.turnToHeading(-90, 2000, {.maxSpeed = 100});
        chassis.moveToPoint(60, 0, 2500, {.forwards = false, .maxSpeed = 100});
        chassis.waitUntilDone();
        piston.set_value(false);
        chassis.moveToPoint(0, 16, 3000, {.forwards = true, .maxSpeed = 70});
        chassis.turnToHeading(90, 1500, {.maxSpeed = 70});

        chassis.moveToPoint(-23, 16, 1500, {.forwards = false, .maxSpeed = 70});
        chassis.waitUntilDone();
        piston.set_value(true);
        chassis.turnToHeading(-90, 2000, {.maxSpeed = 75});
        chassis.waitUntilDone();
        intake.move_absolute(45000, 200);
        conveyor.move_absolute(45000, 200);
        chassis.moveToPoint(-60, 16, 5000, {.forwards = true, .maxSpeed = 70});
        chassis.moveToPoint(-45, 16, 2000, {.forwards = false, .maxSpeed = 100});
        chassis.turnToHeading(-180, 2000, {.maxSpeed = 100});
        chassis.waitUntilDone();
        intake.move_absolute(60000, 200);
        conveyor.move_absolute(6000, 200);
        chassis.moveToPoint(45, 5, 2000, {.forwards = true, .maxSpeed = 100});
        chassis.turnToHeading(-90, 2000, {.maxSpeed = 100});
        chassis.moveToPoint(60, 0, 2500, {.forwards = false, .maxSpeed = 100});
        chassis.waitUntilDone();
        piston.set_value(false);
    }
    if(justMove == true && regularAuton == false) {
        intake.set_zero_position(0);
        conveyor.set_zero_position(0);
        chassis.setPose(0, 0, 180);
        chassis.moveToPoint(0, 30, 700, {.forwards = false, .maxSpeed=70}, true);
        chassis.waitUntilDone();
        pros::delay(500);
        piston.set_value(true);
        pros::delay(1000);
        conveyor.move_absolute(-360, 127);
        pros::delay(1000);
        conveyor.move_absolute(2000, 127);
    }
    if(test == true && regularAuton == false) {
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 48, 100000);
        //chassis.turnToHeading(180, 100000);
    }
    if(red == true && rings == true && test == false && justMove == false && skills == false && regularAuton == true){
        intake.set_zero_position(0);
        conveyor.set_zero_position(0);
        chassis.setPose(0, 0, 0);
        conveyor.move_absolute(-360, 50);
        chassis.moveToPoint(0, -13, 700, {.forwards = false}, true);
        pros::delay(500);
        chassis.turnToHeading(90, 700);
        pros::delay(500);
        chassis.moveToPoint(-10, -15, 500, {.forwards = false}, true);
        conveyor.move_absolute(5000, 127);
        pros::delay(1000);
        chassis.moveToPoint(2, -15, 1000, {.forwards = true}, true);
        pros::delay(1000);
        chassis.turnToHeading(180, 700);
        pros::delay(1000);
        chassis.moveToPoint(2, -9, 1000, {.forwards = false}, true);
        pros::delay(1000);
        chassis.turnToHeading(240, 1000);
        pros::delay(1000);
        piston.set_value(false);
    }
    if(red == true && rings == false && test == false && justMove == false && skills == false && regularAuton == true){
        intake.set_zero_position(0);
        conveyor.set_zero_position(0);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -30, 2000, {.forwards = false, .maxSpeed = 70});
        chassis.waitUntilDone();
        piston.set_value(true);
        pros::delay(500);
        conveyor.move_absolute(10000, 1000);
        pros::delay(500);
        chassis.turnToHeading(-90, 1000, {.maxSpeed = 70});
        chassis.waitUntilDone();
        intake.move_absolute(15000, 500);
        conveyor.move_absolute(15000, 1000);
        chassis.moveToPoint(-25, -32, 2500, {.forwards = true, .minSpeed = 100});
        chassis.moveToPoint(0, -30, 2000, {.forwards = false, .maxSpeed = 70});
        chassis.turnToHeading(315, 3000, {.maxSpeed = 70});
        chassis.moveToPoint(15, -40, 2000, {.forwards = false, .maxSpeed = 70});
    }
    if(red == false && rings == true && test == false && justMove == false && skills == false && regularAuton == true){
        intake.set_zero_position(0);
        conveyor.set_zero_position(0);
        chassis.setPose(0, 0, 0);
        conveyor.move_absolute(-360, 50);
        chassis.moveToPoint(0, -13, 700, {.forwards = false}, true);
        pros::delay(500);
        chassis.turnToHeading(90, 700);
        pros::delay(500);
        chassis.moveToPoint(-10, -15, 500, {.forwards = false}, true);
        conveyor.move_absolute(5000, 127);
        pros::delay(1000);
        chassis.moveToPoint(2, -15, 1000, {.forwards = true}, true);
        pros::delay(1000);
        chassis.turnToHeading(180, 700);
        pros::delay(1000);
        chassis.moveToPoint(2, -9, 1000, {.forwards = false}, true);
        pros::delay(1000);
        chassis.turnToHeading(240, 1000);
        pros::delay(1000);
        piston.set_value(false);
    }
    if(red == false && rings == false && test == false && justMove == false && skills == false && regularAuton == true){
        intake.set_zero_position(0);
        conveyor.set_zero_position(0);
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -30, 2000, {.forwards = false, .maxSpeed = 70});
        chassis.waitUntilDone();
        piston.set_value(true);
        pros::delay(500);
        conveyor.move_absolute(10000, 1000);
        pros::delay(500);
        chassis.turnToHeading(90, 1000, {.maxSpeed = 70});
        chassis.waitUntilDone();
        intake.move_absolute(15000, 500);
        conveyor.move_absolute(15000, 1000);
        chassis.moveToPoint(25, -32, 2500, {.forwards = true, .minSpeed = 100});
        chassis.moveToPoint(0, -30, 2000, {.forwards = false, .maxSpeed = 70});
        chassis.turnToHeading(315, 3000, {.maxSpeed = 70});
        chassis.moveToPoint(-15, -40, 2000, {.forwards = false, .maxSpeed = 70});
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

    // drive setup
    while(setupDrive == false) {
        pros::screen::erase();
        pros::screen::print(TEXT_LARGE, 1, "Rowan Stuff = A"); // prints to brain
        pros::screen::print(TEXT_LARGE, 3, "Jake + Liam Stuff = B"); // prints to brain
        pros::screen::print(TEXT_LARGE, 5, "Skills (Jake Drive) = X"); // prints to brain
        pros::screen::print(TEXT_LARGE, 7, "Contorller Mod Drive = Y"); // prints to brain
        if(controller.get_digital(DIGITAL_A)){
            rowan = true;
            setupDrive = true;
        }
        else if(controller.get_digital(DIGITAL_B)){
            rowan = false;
            setupDrive = true;
        }
        else if(controller.get_digital(DIGITAL_X)){
            rowan = false;
            skills = true;
            setupDrive = true;
            setupColor = true;
            setupSide = true;
            pros::delay(5000);
        }
        else if(controller.get_digital(DIGITAL_Y)){
            modded = true;
            rowan = false;
            setupDrive = true;
        }
        pros::delay(20);
    }

	// auton setup
    while(setupColor == false && setupDrive == true) {
        pros::screen::erase(); // erases brain
        pros::screen::print(TEXT_LARGE, 1, "Red = UP"); // prints to brain
        pros::screen::print(TEXT_LARGE, 3, "Blue = DOWN"); // prints to brain
        pros::screen::print(TEXT_LARGE, 5, "Test = LEFT"); // prints to brain
        pros::screen::print(TEXT_LARGE, 7, "Just Move = RIGHT"); // prints to brain
        
        if(controller.get_digital(DIGITAL_UP)){
            red = true; // sets our auton to red alliance
            setupColor = true; // finishes the second setup
            regularAuton = true;
        }
        else if(controller.get_digital(DIGITAL_DOWN)){
            red = false; // sets our auton to blue alliance
            setupColor = true; // finishes the second setup
            regularAuton = true;
        }
        else if(controller.get_digital(DIGITAL_LEFT)){
            test = true;
            pros::delay(2000);
            setupColor = true;
            setupSide = true;
        }
        else if(controller.get_digital(DIGITAL_RIGHT)){
            justMove = true;
            pros::delay(2000);
            setupColor = true;
            setupSide = true;
        }
        pros::delay(20); // delay to save resources
    }

    while(setupColor == true && setupSide == false && setupDrive == true) {
        pros::screen::erase(); // erases brain
        pros::screen::print(TEXT_LARGE, 1, "Rings = L2 "); // prints to brain
        pros::screen::print(TEXT_LARGE, 3, "Mobile Goal = R2"); // prints to brain
        if(controller.get_digital(DIGITAL_L2)){
            rings = true; // sets our auton to the side with 4 rings
            pros::delay(2000);
            setupSide = true; // finishes the third setup
        }
        else if(controller.get_digital(DIGITAL_R2)){
            rings = false; // sets our auton to the side with an extra mobile goal
            pros::delay(2000);
            setupSide = true; // finishes the third setup
        }
        pros::delay(20); // delay to save resources
    }


    pros::screen::erase(); // erases brain
    
    lv_obj_t * img1 = lv_img_create(lv_scr_act());
    lv_img_set_src(img1, &axolotlFire);
    lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);  
    

    while (setupColor == true && setupSide == true && setupDrive == true) {


        // tank drive control scheme
        if(rowan == false) {
        	int tankLeft = controller.get_analog(ANALOG_LEFT_Y); // the left analog stick controls the left side of the drive train
        	int tankRight = controller.get_analog(ANALOG_RIGHT_Y); // the right analog stick controls the left side of the drive train        
        	chassis.tank(tankLeft, tankRight); // creates a tank drive scheme
        }

        // arcade drive control scheme
        else if(rowan == true) {
            int arcadeLeft = controller.get_analog(ANALOG_LEFT_Y);
            int arcadeRight = controller.get_analog(ANALOG_RIGHT_X);
            chassis.arcade(arcadeLeft, arcadeRight);
        }


        // intake + conveyor control scheme
        if(controller.get_digital(DIGITAL_R1)) {
            intake.move(60); // moves the intake forwards at max speed
            conveyor.move(127); // moves the conveyor forwards at max speed
        }
        else if(controller.get_digital(DIGITAL_R2)) {
            intake.move(-60); // moves the intake forwards at max speed
            conveyor.move(-127); // moves the conveyor backwards at max speed
        }
        else if(controller.get_digital(DIGITAL_UP)){
            intake.move(60);
        }
        else {
            intake.move(0); // stops the intake from moving
            conveyor.move(0); // stops the conveyor from moving
        }


        // clamp control scheme (new)
        if(rowan == false && modded == false){
            if(controller.get_digital(DIGITAL_A)){
              piston.set_value(false);
            }
            else if(controller.get_digital(DIGITAL_B)){
                piston.set_value(true);
            }
        }
        else if(rowan == true && modded == false){
            if(controller.get_digital(DIGITAL_L2)){
                piston.set_value(false);
            }
            else if(controller.get_digital(DIGITAL_L1)){
                piston.set_value(true);
            }
        }
        else if(modded == true){
            if(controller.get_digital(DIGITAL_DOWN)){
                piston.set_value(false);
            }
            else if(controller.get_digital(DIGITAL_B)){
                piston.set_value(true);
            }
        }


        /* clamp control scheme (old)
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
        */
       
        // lift control scheme

        if(rowan == false) {
            if(controller.get_digital(DIGITAL_L1)){
                lift.set_value(true);
            }
            else if(controller.get_digital(DIGITAL_L2)){
                lift.set_value(false);
            }
        }

        else if(rowan == true) {
            if(controller.get_digital(DIGITAL_UP)){
                lift.set_value(true);
            }
            else if(controller.get_digital(DIGITAL_DOWN)){
                lift.set_value(false);
            }
        }


        // wing control scheme
        if(modded == false){
            if(controller.get_digital(DIGITAL_LEFT)){
                wing.set_value(true);
            }
            else if(controller.get_digital(DIGITAL_RIGHT)){
                wing.set_value(false);
            }
        }
        else if(modded == true){
            if(controller.get_digital(DIGITAL_RIGHT)){
                wing.set_value(true);
            }

            else if(controller.get_digital(DIGITAL_Y)){
                wing.set_value(false);
            }
        }


        pros::delay(20); // delay to save resources
    }
}
