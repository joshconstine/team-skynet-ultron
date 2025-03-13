#include <Pololu3piPlus32U4.h>
#include "PIDcontroller.h"
using namespace Pololu3piPlus32U4;

PIDcontroller::PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i) {
  /*Initialize values by copying and pasting from PD controller, then declaring for
  the three new variables.*/
  _kp = kp;
  _kd = kd;
  _minOutput = minOutput;
  _maxOutput = maxOutput;
  
  // Initialize PID-specific values
  _ki = ki;
  _clamp_i = clamp_i;
  _integral = 0;
  _previousError = 0;
  _previousTime = millis();
}
double PIDcontroller::update(double value, double target_value) {
  double error = target_value - value;

  unsigned long currentTime = millis();
  double deltaTime = (currentTime - _previousTime) / 1000.0;  // Convert to seconds

  double Pout = _kp * error;

  _integral += error * deltaTime;
  _integral = constrain(_integral, -_clamp_i, _clamp_i);  // Clamp the accumulated error
  double Iout = _ki * _integral;

  double derivative = (error - _previousError) / deltaTime;
  double Dout = _kd * derivative;

  // Calculate total output and clamp to min and max output limits
  double output = Pout + Iout + Dout;
  output = constrain(output, _minOutput, _maxOutput);

  // Logging for debugging
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | Pout: ");
  Serial.print(Pout);
  Serial.print(" | Iout: ");
  Serial.print(Iout);
  Serial.print(" | Dout: ");
  Serial.print(Dout);
  Serial.print(" | Output: ");
  Serial.println(output);

  _previousError = error;
  _previousTime = currentTime;

  return output;  // Ensure the function returns the computed output
}
