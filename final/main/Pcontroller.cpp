#include <Pololu3piPlus32U4.h>
#include "Pcontroller.h"
using namespace Pololu3piPlus32U4;

Pcontroller::Pcontroller(float kp, double minOutput, double maxOutput) {
    _kp = kp;
    _minOutput = minOutput;
    _maxOutput = maxOutput;
}

double Pcontroller::update(double value, double target_value){
  //Controller math here
  //Hint: Need to return actuator controller value (_clampOut)
  _error = target_value - value;  
  
  Serial.print("error: ");
  Serial.print(_error);

  _Pout = _kp * _error;     

    Serial.print("  Pout before contrain: ");
  Serial.print(_Pout);       

  _Pout = constrain(_Pout, _minOutput, _maxOutput);  


    Serial.print("  Pout after contrain: ");
  Serial.print(_Pout);   
  return _Pout;
}
