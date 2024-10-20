/*#pragma once
#include "lib/TaskWrapper.hpp"
#include "lib/point.hpp"
#include "lib/pid.h"
#include "lib/trackwheel.h"
#include "pros/motors.h"
#include <memory>

namespace lib {

class Chassis : public ryan::TaskWrapper{

private:
  enum class DriveState { IDLE, MOVING };
  std::shared_ptr<pros::MotorGroup> leftMotors;
  std::shared_ptr<pros::MotorGroup> rightMotors;
  std::shared_ptr<pros::Imu> imu;
  std::shared_ptr<lib::TrackingWheel> track;

  Point currentPose = Point(0, 0, 0);

  DriveState state;
  float headingTarget;

  const int rpm;
  const double wheel;

  double angleWrap(double angle) {
    while (angle > 360) {
      angle -= 360;
    }
    while (angle < -360) {
      angle += 360;
    }
    return angle;
  }

public:
  bool blueSide = false;

  DriveState getState() { return state; }

  // constructor
  Chassis(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, pros::Imu *imu, lib::TrackingWheel *track, int rpm, double wheel)
  : leftMotors(leftMotors), rightMotors(rightMotors), imu(imu), track(track), rpm(rpm), wheel(wheel) 
  {
    leftMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    rightMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    state = DriveState::IDLE;
  }

  //tracking

  
  // tracking
  void loop() override;
  Point getPose(bool radians = false);
  void setPose(Point newPose, bool radians = false);

  // driver functions
  int inputCurve(int input, double t = 1);
  void arcadeMod(double forward, double turn, double curve, int speedThreshold, int speedCap);
  void arcade(double forward, double turn, std::vector<double> curves);
  void arcade(double forward, double turn);
  void tank(double left, double right, std::vector<double> curves = {0, 0});



  void waitUntilFinished();



  // relative pid movements
  void move(float target, PID linearPid, PID headingPid, float maxSpeed = 110, bool async = false);

  void turn(double target, PID turningPid, float maxSpeed = 127, bool async = false);

  void swing(double target, bool side, float multiplier, PID turningPid, float maxSpeed = 127, bool async = false);
 


  // odom movements
  void moveToPoint(float x, float y, PID linearPid, PID headingPid, float maxSpeed = 127, bool async = false);

};
} // namespace lib*/