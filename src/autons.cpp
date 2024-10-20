#include "lib/lift.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.h"
#include "autons.h"

void exit_condition_defaults() {
  turning.exit_condition_set(150, 1, 300, 3, 500, 500);
  linear.exit_condition_set(150, .25, 300, .75, 500, 500);
  swing.exit_condition_set(150, 1, 300, 3, 500, 500);
}

void bluePositive() {
}