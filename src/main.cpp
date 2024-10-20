#include "autons.h"
#include "lib/lift.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robodash/api.h"
#include "robotconfig.h"
#include <string>
#include "lemlib/api.hpp"

// ================================= Views ================================= //

// Create robodash selector
rd::Selector selector({{"Skills", bluePositive}});

// ================================= Lem ================================= //

pros::Rotation horizontal_encoder(20);
pros::Rotation vertical_encoder(19);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2);

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
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
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


// ========================= Competition Functions ========================= //

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

  exit_condition_defaults();

  lift.startTask();
}

void disabled() {
  clamp.retract();
  lift.setState(lib::LiftState::Stored);
}

void competition_initialize() {}

void autonomous() {
  console.focus();
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


  bluePositive();
}

void opcontrol() {

  float liftTarget = -1;


  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  while (true) {

        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

    intake.move((controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))   ? 127
                : (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) ? -127
                                                                          : 0);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      doinker.toggle();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      sorter.toggle();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      clamp.toggle();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      lift.itterateState(1);
    } else if (controller.get_digital_new_press(
                   pros::E_CONTROLLER_DIGITAL_L2)) {
      lift.itterateState(0);
    }

    pros::delay(15);
  }
}
