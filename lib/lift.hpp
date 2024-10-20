#pragma once
#include "StateMachine.hpp"
#include "lib/TaskWrapper.hpp"
#include "liftPID.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include <memory>

namespace lib {

enum class LiftState {Stored, Recieve, Score};

  //pid constant struct
  struct PIDConstants {
    double kP;
    double kI;
    double kD;
  };

class Lift : public StateMachine<LiftState, LiftState::Stored>, public ryan::TaskWrapper {

private:
  std::shared_ptr<pros::MotorGroup> motors;
  const float DOWN_ANGLE = 0;
  const float MID_ANGLE = 34;
  const float UP_ANGLE = 150 ;

  const float gearRatio;

  const PIDConstants constants;

  double target;

  liftPID liftpid = liftPID(constants.kP, constants.kI, constants.kD);

public:
  Lift(pros::MotorGroup *motors, double gearRatio, PIDConstants constants) : motors(motors), gearRatio(gearRatio), constants(constants) {
    motors->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  void loop() override;

  float getAngle();
  void itterateState(bool delta);
  
};

} // namespace lib
// namespace lib