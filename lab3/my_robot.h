#ifndef my_robot_h
#define my_robot_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;



// The following actuators are supported:
// • 2 Motors
// • LEDs (4 binary status LED)
// • OLED display
// • Buzzer speaker (playing songs and simple
// sounds) The following sensors are available:
// • Wheel encoders
// • 5 line sensors (downward facing IR sensors)
// • 2 bump sensors
// • IMU (inertial measurement unit, includes 3-axis accelerometer, 3-axis gyroscope, and 3-axis
// magnetometer)
// • 5 Buttons
// • Power-related sensors (voltage, currents, battery capacity

class MyRobot{
  public:
    MyRobot(float baseSpeed = 0.4); // Constructor with default base speed
    void forward(float distance, float speed = -1);
    void backward(float distance, float speed = -1);
    void turn_left(float duration, float speed = -1);
    void turn_right(float duration, float speed = -1);
    void turn_left_and_forward(float duration, float speed = -1, double offset = .1);
    void turn_right_and_forward(float duration, float speed = -1, double offset = .1);
    void halt();
    
  private:
    float baseSpeed;  // Base speed in m/s
    Motors motors;
    short int convertSpeed(float speed);  // Converts m/s to motor input speed
    
};

#endif