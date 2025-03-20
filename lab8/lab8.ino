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

  // Clear old values (optional for your flow)
  for (int i = 0; i < 5; i++)
  {
    lineSensorValues[i] = 0;
  }

  // --- Turn 90° LEFT ---
  Serial.println("Turning 90° LEFT...");
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  delay(3500); // Adjust this until you get a real 90° turn
  motors.setSpeeds(0, 0);  
  Serial.println("Finished 90° LEFT turn. Reading sensors...");
  
  // Read and print sensor values at 90° left orientation
  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // --- Return to CENTER from Left (another 90° turn right) ---
  Serial.println("Returning to CENTER orientation...");
  motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
  delay(3500); 
  motors.setSpeeds(0, 0);  
  Serial.println("Returned to CENTER. Reading sensors...");

  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // --- Turn 90° RIGHT from center ---
  Serial.println("Turning 90° RIGHT...");
  motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
  delay(3500); // Adjust this for a true 90° turn
  motors.setSpeeds(0, 0);
  Serial.println("Finished 90° RIGHT turn. Reading sensors...");

  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // --- Return to CENTER from Right (another 90° turn left) ---
  Serial.println("Returning to CENTER orientation...");
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  delay(3500); 
  motors.setSpeeds(0, 0);
  Serial.println("Returned to CENTER. Reading sensors...");

  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // Final stop
  motors.setSpeeds(0, 0);

  Serial.println("Calibration finished.");
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
  int lineCenter = 2000;
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
