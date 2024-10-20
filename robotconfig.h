#pragma once
#include "lib/lift.hpp"
#include "lib/chassis.h"
#include "lib/liftPID.h"
#include "lib/lights.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"


inline pros::MotorGroup leftMotors({11,-12,13}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

inline pros::MotorGroup rightMotors({-3,5,4}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

inline pros::Imu imu(11);

inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

inline pros::MotorGroup intake({2});

inline pros::MotorGroup armMotors({7, -16});
inline lib::Lift lift(&armMotors,  12.0/60, {1.2, 0, 0.6});

inline pros::adi::Pneumatics doinker('H', false);
inline pros::adi::Pneumatics clamp('G', false);
inline pros::adi::Pneumatics sorter('F', false);

inline liftPID linear(10,0.0, 34);
inline liftPID heading(2.875, 0, 20, 0);
inline liftPID turning(2.875, 0, 10, 0);
inline liftPID swing(.5, 0.0, 25, 7.5);

inline rd::Console console;

//LemLib

// horizontal tracking wheel