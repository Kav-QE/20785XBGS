#include "lemlib/api.hpp"
#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robodash/views/selector.hpp"

// ================================ Setup ================================ //
// Motor Config
pros::MotorGroup leftDrivebase({-13, -14, 15}, pros::MotorGearset::blue);
pros::MotorGroup rightDrivebase({16, 17, -18}, pros::MotorGearset::blue);

pros::Motor upperIntake(2, pros::MotorGearset::blue);
pros::Motor lowerIntake(9, pros::MotorGearset::green);

pros::Motor arm(1, pros::MotorGearset::green);

// Pneumatics Config
pros::adi::Pneumatics clamp{'B', true};
pros::adi::Pneumatics doinker{'A', false};
pros::adi::Pneumatics climb{'F', false};

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drivebase
lemlib::Drivetrain drivebase(&leftDrivebase,
                             &rightDrivebase,
                             lemlib::Omniwheel::NEW_275, 
                             12.5, 
                             450, 
                             8);

// Input Scaling
lemlib::ExpoDriveCurve throttleCurve(5,
                                    5,
                                    1.04);

lemlib::ExpoDriveCurve steerCurve(5,
                                    5,
                                    1.04);

// Sensors
pros::IMU imu(20);

// Tracking
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// PID
lemlib::ControllerSettings lateralController(2,
                                            0,
                                            10,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0);

lemlib::ControllerSettings angularController(2,
                                            0,
                                            10,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0);

// Chassis
lemlib::Chassis chassis(drivebase, 
                        lateralController, 
                        angularController, 
                        sensors, 
                        &throttleCurve, 
                        &steerCurve);

// Robot Functions
void lowerForward() {
    lowerIntake.move_velocity(-600);
}
void lowerReverse() {
    lowerIntake.move_velocity(600);
}
void lowerStop() {
    lowerIntake.brake();
}

void upperForward() {
    upperIntake.move_velocity(-600);
}
void upperReverse() {
    upperIntake.move_velocity(600);
}
void upperStop() {
    upperIntake.brake();
}

void armForward() {
    arm.move_velocity(-100);
}
void armReverse() {
    arm.move_velocity(100);
}
void armStop() {
    arm.brake();
}

void allianceMacro() {
    arm.move_relative(200, 200);
    arm.move_relative(350, 200);
    arm.move_relative(-150, 200);
}

void climbToggle() {
    climb.toggle();
}
void mogoToggle() {
    clamp.toggle();
}
void doinkerToggle() {
    doinker.toggle();
}



// ================================ Autonomous ================================ //
// Autons
// 7 Points
void redAWP() {}
void blueAWP() {}

// 5 Points
void redRingRush() {}
void blueRingRush() {}

// 6 Points
void redSouth() {}
void blueSouth() {}

void PIDTest() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turn(90, 100000);
}



// Selector
rd::Selector selector({
    {"Red AWP", &redAWP},
    {"Blue AWP", &blueAWP},
    {"Red Ring Rush", &redRingRush},
    {"Blue Ring Rush", &blueRingRush},
    {"Red South", &redSouth},
    {"Blue South", &blueSouth}
});



// ================================ Field Control ================================ //
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
}

void disabled() {}

void competition_initialize() {
    // selector.focus();
}

void autonomous() {
	PIDTest();

    // selector.run_auton();
}

void opcontrol() {
    while (true) {
        arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        // Drivebase
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX, false, 0.62);

        // Pneumatics
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			mogoToggle();
		}
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            doinkerToggle();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            climbToggle();
        }

        // Intake
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            lowerForward();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            upperForward();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            lowerReverse();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            upperReverse();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            upperStop();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            lowerStop();
        }

        // Arm
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            allianceMacro();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            armForward();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            armReverse();
        }
        else {
            armStop();
        }

        // Memory
        pros::delay(25);
    }
}
