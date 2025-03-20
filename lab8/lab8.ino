#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
//#include "PDcontroller.h" //Uncomment after you import your PDcontroller files

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line 2.0    // Adjust this for better response
#define kd_line 1.0    // Adjust this for smoothness

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);

// Recommended Variables

// Calibration
int calibrationSpeed = 30;  // Speed for calibration
unsigned int lineSensorValues[5];  // Stores sensor readings

// Line Following
int lineCenter;  // The center position for the line
int16_t robotPosition;  // This will be updated to keep track of robot's position on the line

void calibrateSensors()
{
    Serial.println("Starting sensor calibration...");

  // Implement calibration for IR Sensors
  // Move the robot in a sweeping motion to calibrate sensors.
  
  // Example: Turn left to calibrate one side, then turn right to calibrate the other
  for (int i = 0; i < 5; i++) {
    lineSensorValues[i] = 0;  // Start with all sensors reading 0.
  }
  
  // Calibration sweep: Turn left, right, and read the values
  for (int i = 0; i < 5; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
    lineSensorValues[i] = analogRead(i);  // Read the sensor value for each sensor
  }
  // Optionally, use these values to compute a baseline sensor calibration for the robot.

  // Move the robot a bit after calibration
  motors.setSpeeds(0, 0); // Stop motors after calibration
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("Robot is initializing...");

  calibrateSensors();  // Calibrate the line sensors
  
  // Initialize line sensors
  // lineSensors.init();
}
void loop() {
  // Read the line sensor values
  lineSensors.read(lineSensorValues);

  // Calculate the position of the line using the weighted sum method
  int lineCenter = 0;
  int totalWeight = 0;

  for (int i = 0; i < 5; i++) {
    lineCenter += lineSensorValues[i] * i;
    totalWeight += lineSensorValues[i];
  }

  // Avoid division by zero
  if (totalWeight != 0) {
    lineCenter /= totalWeight;
  } else {
    lineCenter = 2; // Default to center position if no sensor detects the line
  }

  // Calculate robot's position relative to the line center (ideal is 0, meaning perfectly on the line)
  int robotPosition = lineCenter - 2;  // Adjust the range to [-2, 2] for a 5-sensor setup
  
 // Print out the computed line center and robot position
  Serial.print("lineCenter: ");
  Serial.print(lineCenter);
  Serial.print(" | robotPosition: ");
  Serial.println(robotPosition);

  // Use the PD controller to compute the control output
  double controlOutput = pd_line.update(robotPosition, 0);  // Target value is 0

  // Print out the PD controller output
  Serial.print("controlOutput: ");
  Serial.println(controlOutput);

  // Adjust the robot's speed based on the control output
  int leftSpeed = baseSpeed + controlOutput;
  int rightSpeed = baseSpeed - controlOutput;

  // Ensure the robot's speed stays within bounds
  leftSpeed = constrain(leftSpeed, minOutput, maxOutput);
  rightSpeed = constrain(rightSpeed, minOutput, maxOutput);

 // Print final motor speeds
  Serial.print("Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);

  
  // Set the motor speeds
  motors.setSpeeds(leftSpeed, rightSpeed);
}
