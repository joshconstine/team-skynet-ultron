#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

PDcontroller::PDcontroller(float kp, float kd, double minOutput, double maxOutput) {
  // initialize the private varaibles from Pcontroller.h here
  _kp = kp;
  _kd = kd;
  _minOutput = minOutput;
  _maxOutput = maxOutput;
}

double PDcontroller::update(double value, double target_value){
  //Controller math here
  /*Hints: To add damping (derivative), you must have something to
           keep track of time for the rate of change.
           
           Also note that the first time PD controller is ran, we only have
           the P component, so consider using an if-else statement.

           Again, you need to return actuator controller value (_clampOut)
  */

  double error = target_value - value;
  unsigned long currentTime = millis();
  double deltaTime = (currentTime - _previousTime) / 1000.0; // Convert to seconds

  double P = _kp * error;
  double D = 0;
  
  // If not the first iteration, calculate derivative component
  if (_previousTime != 0) {
      double deltaError = error - _previousError;
      D = _kd * (deltaError / deltaTime);
  }

  double output = P + D;
  
  // Clamping output to max and min values
  if (output > _maxOutput) output = _maxOutput;
  if (output < _minOutput) output = _minOutput;

  // Store current values for next iteration
  _previousError = error;
  _previousTime = currentTime;

  return output;
}
