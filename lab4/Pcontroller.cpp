#include <Pololu3piPlus32U4.h>
#include "Pcontroller.h"
using namespace Pololu3piPlus32U4;

Pcontroller::Pcontroller(float kp, double minOutput, double maxOutput) {
  // initialize the private varaibles from Pcontroller.h here
}

double Pcontroller::update(double value, double target_value){
  //Controller math here
  //Hint: Need to return actuator controller value (_clampOut)
}
