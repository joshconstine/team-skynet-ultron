#include <Pololu3piPlus32U4.h>
#include "PIDcontroller.h"
using namespace Pololu3piPlus32U4;

PIDcontroller::PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i) {
  /*Initialize values by copying and pasting from PD controller, then declaring for
  the three new variables.*/
}

double PIDcontroller::update(double value, double target_value){
  /*Now copy and paste your PD controller. To implement I component,
  keep track of accumulated error, use your accumulated error in the constrain
  function for the integral, multiply ki by your integral, then add your p, d,
  and i components.
  
  Note: Do not just put all of the integral code at the end of PD component. Think
  about step by step how you can integrate these parts into your PDController
  code.*/
  
}
