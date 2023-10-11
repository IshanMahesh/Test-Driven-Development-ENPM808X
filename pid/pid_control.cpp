/**
 * @file pid_control.cpp
 * @author Ishan Mahesh Kharat (Driver)
 * @author Akashkumar Parmar (Navigator)
 * @brief This method is used to initalize the private variables used ,
 * constructor and method
 * @version 0.1
 * @date 2023-10-09
 * @copyright Copyright (c) 2023
 */

#include "pid_control.hpp"

#include <cmath>
#include <iostream>

/**
 * @brief The method is used to compute the new velocity value based on
 * target setpoint and actual velocity and considering the pid constants
 * initialized.
 * @param target_setpoint
 * @param actual_velocity
 * @return double
 */
double PID::PIDCONTROLLER::compute(double target_setpoint,
                                   double actual_velocity) {
  double previous_error = 0;
  double current_error = 0;
  double total_error = 0;
  double time = 10;
  double final_velocity;
  for (int dt = 1; dt <= time; dt++) {
    // calculating the error
    current_error = target_setpoint - actual_velocity;

    // calculating the derivative error
    double derivative_error = abs(current_error - previous_error);

    // calculating the integral error
    double integral_error = total_error * dt;

    // Updating previous error
    previous_error = current_error;

    // updating total error
    total_error += current_error;

    // calculating the total output
    double output =
        (kp * current_error) + (kd * derivative_error) + (ki * integral_error);

    final_velocity += output;
  }
  return final_velocity;
}
