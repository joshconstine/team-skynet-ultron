#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h" 

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;
Servo servo;

Sonar sonar(4);

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line 2.0    // Proportional gain for line following
#define kd_line 1.0    // Derivative gain for line following
#define kp_obs 55      // Proportional gain for obstacle avoidance (from reference)
#define kd_obs 10      // Derivative gain for obstacle avoidance (from reference)

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);
PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);

//Recommended Variables

//Calibration
int calibrationSpeed;
unsigned int lineSensorValues[5];
unsigned int lineDetectionValues[5];

//Line Following
int lineCenter = 2000;
int16_t robotPosition;
bool isOnBlack;

//Wall Following
int PDout;
float wallDist;
int distFromWall = 10;

// State machine states
enum RobotState {
  LINE_FOLLOWING,
  OBSTACLE_DETECTED,
  ROTATING_SERVO,
  ROTATING_ROBOT,
  WAITING,
  WALL_FOLLOWING,
  RETURNING_TO_LINE
};

RobotState currentState = LINE_FOLLOWING;

// Obstacle avoidance parameters
#define OBSTACLE_DISTANCE 15  // Distance in cm to detect obstacles
#define WALL_DISTANCE 10.0    // Distance to maintain from wall while wall following
#define TURN_SPEED 100       // Speed for turning
#define FORWARD_SPEED 400    // Base speed for forward movement (from reference)

void calibrateSensors()
{
  Serial.println("Starting sensor calibration...");

  // Clear old values
  for (int i = 0; i < 5; i++)
  {
    lineSensorValues[i] = 0;
  }

  // --- Turn 90° LEFT ---
  Serial.println("Turning 90° LEFT...");
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  delay(3500);
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

  // --- Return to CENTER from Left ---
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
  delay(3500);
  motors.setSpeeds(0, 0);
  Serial.println("Finished 90° RIGHT turn. Reading sensors...");

  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // --- Return to CENTER from Right ---
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

void lineFollowing()
{
  // Read the line sensor values
  lineSensors.read(lineSensorValues);

  // Calculate the position of the line using the weighted sum method
  int lineCenter = 0;
  int totalWeight = 0;

  for (int i = 0; i < 5; i++) {
    lineCenter += lineSensorValues[i] * i;
    totalWeight += lineSensorValues[i];
  }

  robotPosition = lineCenter / 5;
  
  // Print out the computed line center and robot position
  Serial.print("lineCenter: ");
  Serial.print(lineCenter);
  Serial.print(" | robotPosition: ");
  Serial.println(robotPosition);

  // Use the PD controller to compute the control output
  double controlOutput = pd_line.update(robotPosition, 2700);  // Target value is 2700

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

void wallFollowing()
{
  wallDist = sonar.readDist();
  
  // Calculate error from desired wall distance
  double error = wallDist - WALL_DISTANCE;
  
  // Get PD controller output
  double pdOutput = pd_obs.update(error, 0);
  
  // Set motor speeds with base speed
  int leftSpeed = baseSpeed + pdOutput;
  int rightSpeed = baseSpeed - pdOutput;
  
  // Ensure speeds stay within bounds
  leftSpeed = constrain(leftSpeed, minOutput, maxOutput);
  rightSpeed = constrain(rightSpeed, minOutput, maxOutput);
  
  // Debug printing
  Serial.print("Wall Distance: ");
  Serial.print(wallDist);
  Serial.print(" | PDout: ");
  Serial.print(pdOutput);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);
  
  motors.setSpeeds(leftSpeed, rightSpeed);
}

void detectBlackLine()
{
  lineSensors.read(lineDetectionValues);
  const int blackThreshold = 1500;
  
  for (int i = 0; i < 5; i++) {
    if (lineDetectionValues[i] > blackThreshold) {
      isOnBlack = true;
      return;
    }
  }
  isOnBlack = false;
}

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90); // turn servo forward for line following
  delay(2000);

  // calibrateSensors();
}

void loop() {
  // Read sonar distance
  float sonarDist = sonar.readDist();
  
  // State machine implementation
  switch (currentState) {
    case LINE_FOLLOWING:
      Serial.println("State: LINE_FOLLOWING");
      servo.write(90); // Ensure sonar is facing forward
      lineFollowing();
      
      // Check for obstacles
      if (sonarDist < OBSTACLE_DISTANCE) {
        currentState = OBSTACLE_DETECTED;
        motors.setSpeeds(0, 0); // Stop
      }
      break;
      
    case OBSTACLE_DETECTED:
      Serial.println("State: OBSTACLE_DETECTED");
      motors.setSpeeds(0, 0); // Stop
      currentState = ROTATING_SERVO;
      break;
      
    case ROTATING_SERVO:
      Serial.println("State: ROTATING_SERVO");
      servo.write(180); // Turn sonar to side
      delay(500); // Wait for servo to rotate
      currentState = ROTATING_ROBOT;
      break;
      
    case ROTATING_ROBOT:
      Serial.println("State: ROTATING_ROBOT");
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      delay(800); // Rotate 90 degrees right
      motors.setSpeeds(0, 0);
      currentState = WAITING;
      break;
      
    case WAITING:
      Serial.println("State: WAITING");
      delay(1000); // Wait 1 second before starting wall following
      currentState = WALL_FOLLOWING;
      break;
      
    case WALL_FOLLOWING:
      Serial.println("State: WALL_FOLLOWING");
      wallFollowing();
      
      // Check for black line to return to line following
      detectBlackLine();
      if (isOnBlack) {
        motors.setSpeeds(0, 0); // Stop before rotating servo
        servo.write(90); // Turn sonar back to forward for line following
        delay(500); // Wait for servo to rotate
        currentState = LINE_FOLLOWING;
      }
      break;
  }
  
  delay(50); // Small delay to prevent overwhelming the system
}

//Recommended helper functions
//Uncomment if you choose to implement these functions, but also
//feel free to create your own solutions!

/*
void lineFollowing()
{
  //From lab 8
}

void wallFollowing()
{
  //Hint: Your robot shouldn't only be following the wall. It should also be looking
  //      for something else while following the wall.
}

void detectBlackLine()
{
  lineSensors.read(lineDetectionValues);

    // Threshold value to detect black (adjust based on calibration)
    const int blackThreshold = 1500; 

    // Check if the robot is on a black square
    for (int i = 0; i < 5; i++) {
        Serial.println(lineDetectionValues[i]);
        if (lineDetectionValues[i] > blackThreshold) {
            //#TODO If using this function, decide what to do 
            //      if black line is detected again
        }
    }
}
*/

// with this setup, when the robot switches to from line, to wall following. It is fucked up. When we detect the wall, we need to stop. rotate the servoleft, rotate the robot right. Then wait 1 sec. and begin wall following.