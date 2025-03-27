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
#define kp_obs 0.8     // Proportional gain for obstacle avoidance
#define kd_obs 0.2     // Derivative gain for obstacle avoidance

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
  TURNING,
  WALL_FOLLOWING,
  RETURNING_TO_LINE
};

RobotState currentState = LINE_FOLLOWING;

// Obstacle avoidance parameters
#define OBSTACLE_DISTANCE 15  // Distance in cm to detect obstacles
#define WALL_DISTANCE 10     // Distance to maintain from wall while wall following
#define TURN_SPEED 100       // Speed for turning
#define FORWARD_SPEED 100    // Base speed for forward movement

void calibrateSensors()
{
  // Copy your calibrateSensors() function from lab 8
  Serial.println("Starting sensor calibration...");

  // Clear old values
  for (int i = 0; i < 5; i++)
  {
    lineSensorValues[i] = 0;
  }

  // Turn 90° LEFT
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

  // Return to CENTER from Left
  Serial.println("Returning to CENTER orientation...");
  motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
  delay(3500); 
  motors.setSpeeds(0, 0);  
  Serial.println("Returned to CENTER. Reading sensors...");
}

void lineFollowing()
{
  lineSensors.read(lineSensorValues);
  
  // Calculate weighted average position
  long sum = 0;
  long sumWeight = 0;
  
  for (int i = 0; i < 5; i++) {
    sum += (long)lineSensorValues[i] * (i * 1000);
    sumWeight += lineSensorValues[i];
  }
  
  // Calculate position (0-4000)
  robotPosition = sumWeight == 0 ? 2000 : sum / sumWeight;
  
  // Calculate error from center
  double error = robotPosition - lineCenter;
  
  // Get PD controller output
  double pdOutput = pd_line.update(error, 0);
  
  // Set motor speeds
  int leftSpeed = baseSpeed + pdOutput;
  int rightSpeed = baseSpeed - pdOutput;
  
  motors.setSpeeds(leftSpeed, rightSpeed);
}

void wallFollowing()
{
  wallDist = sonar.readDist();
  
  // Calculate error from desired wall distance
  double error = wallDist - WALL_DISTANCE;
  
  // Get PD controller output
  double pdOutput = pd_obs.update(error, 0);
  
  // Set motor speeds
  int leftSpeed = baseSpeed + pdOutput;
  int rightSpeed = baseSpeed - pdOutput;
  
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
  servo.write(90); // turn servo forward
  delay(2000);

  calibrateSensors();
}

void loop() {
  // Read sonar distance
  float sonarDist = sonar.readDist();
  
  // State machine implementation
  switch (currentState) {
    case LINE_FOLLOWING:
      Serial.println("State: LINE_FOLLOWING");
      lineFollowing();
      
      // Check for obstacles
      if (sonarDist < OBSTACLE_DISTANCE) {
        currentState = OBSTACLE_DETECTED;
        motors.setSpeeds(0, 0); // Stop
      }
      break;
      
    case OBSTACLE_DETECTED:
      Serial.println("State: OBSTACLE_DETECTED");
      // Turn right to avoid obstacle
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      delay(1000);
      currentState = WALL_FOLLOWING;
      break;
      
    case WALL_FOLLOWING:
      Serial.println("State: WALL_FOLLOWING");
      wallFollowing();
      
      // Check for black line to return to line following
      detectBlackLine();
      if (isOnBlack) {
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
