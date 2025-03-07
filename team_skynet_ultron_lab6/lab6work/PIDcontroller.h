#ifndef PIDcontroller_h
#define PIDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PIDcontroller{
  public:
    PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i);
    double update(double value, double target_value);
    
  private:
    /*Add variables from your PDController, then add varaibles for ki, your clamp
    for i component, and accumulated error.*/
    float _kp, _kd;
    double _minOutput, _maxOutput;
    double _previousError = 0;
    unsigned long _previousTime = 0;

    // Additional variables for PIDController
    float _ki;                  // Integral gain
    double _clamp_i;            // Clamp for integral term
    double _integral = 0;    

};

#endif
