#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4BumpSensors.h>
#include <Pololu3piPlus32U4Buttons.h>
#include <Pololu3piPlus32U4Buzzer.h>
#include <Pololu3piPlus32U4Encoders.h>
#include <Pololu3piPlus32U4IMU.h>
#include <Pololu3piPlus32U4IMU_declaration.h>
#include <Pololu3piPlus32U4LCD.h>
#include <Pololu3piPlus32U4LineSensors.h>
#include <Pololu3piPlus32U4Motors.h>
#include <Pololu3piPlus32U4OLED.h>


#include <Servo.h>
#include "sonar.h"
#include "Pcontroller.h"
#include "PDcontroller.h" 
#include "my_robot.h"



using namespace Pololu3piPlus32U4;

// Robot states
enum RobotState {
  WANDER,
  CELEBRATION,
  RETURN_HOME
};

// Global variables
RobotState currentState = WANDER;
int trashCount = 0;
bool atHome = false;

// Speed constants
const float BASE_SPEED = 75;  
const float TURN_SPEED = 75;  
const float WALL_SPEED = 75;  

#define minOutput -100
#define maxOutput 100


// Sensor arrays
uint16_t lineSensorValues[5];
uint16_t lineDetectionValues[5];
bool isOnBlack = false;

// Wall following variables
double wallDist = 0;
const double WALL_DISTANCE = 6.0; // cm
const int baseSpeed = 75;
const int calibrationSpeed = 100;
#define kp_obs 6     // Proportional gain for obstacle avoidance
#define kd_obs 3 

// Robot components
LineSensors lineSensors;
MyRobot robot(BASE_SPEED); // Initialize with base speed
Servo servo;
Buzzer buzzer;
OLED display;
BumpSensors bumpSensors;
Sonar sonar(4);

PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);

// Constants
const int CELEBRATION_DURATION = 3000; // ms
const int SPIN_SPEED = 100; // PWM value
const int WANDER_SPEED = 100; // PWM value
const int MAX_TRASH = 3;

void setup() {
  Serial.begin(9600);
  

  bumpSensors.calibrate();
  display.clear();
  display.print("Trash: 0");
  
  // Calibrate sensors
  calibrateSensors();
}

void loop() {
  // State machine
  switch (currentState) {
    case WANDER:
      wanderState();
      break;
    case CELEBRATION:
      celebrationState();
      break;
    case RETURN_HOME:
      returnHomeState();
      break;
  }
}

void wanderState() {

    //check if on black line
    detectBlackLine();

  // Check for trash (using bump sensors as example)
  if (isOnBlack) {
    currentState = CELEBRATION;
    return;
  }

  // Check if we've found all trash
  if (trashCount >= MAX_TRASH) {
    currentState = RETURN_HOME;
    return;
  }

  // do wall following 
  wallFollowing();
  
  delay(400);

}

void celebrationState() {
  // Stop robot
  robot.halt();

  // Spin 360 degrees
  robot.turn_right(2.0, TURN_SPEED); // Turn right for 2 seconds at turn speed

  // Beep
  buzzer.play("L16 cdegreg4");

  // Update display
  display.clear();
  display.print("Trash: ");
  display.print(trashCount);
  
  // Increment trash count and return to wander
  trashCount++;
  isOnBlack = false;
  currentState = WANDER;
}

void returnHomeState() {
  // For now, just wander until we find home
  wallFollowing();
  delay(400);
}

void calibrateSensors() {
  Serial.println("Starting sensor calibration...");

  // Clear old values
  for (int i = 0; i < 5; i++) {
    lineSensorValues[i] = 0;
  }

  // --- Turn 90° LEFT ---
  Serial.println("Turning 90° LEFT...");
  robot.turn_left(1.0, TURN_SPEED); // Turn left for 1 second at turn speed
  robot.halt();
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
  robot.turn_right(1.0, TURN_SPEED); // Turn right for 1 second at turn speed
  robot.halt();
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
  robot.turn_right(1.0, TURN_SPEED); // Turn right for 1 second at turn speed
  robot.halt();
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
  robot.turn_left(1.0, TURN_SPEED); // Turn left for 1 second at turn speed
  robot.halt();
  Serial.println("Returned to CENTER. Reading sensors...");

  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // Final stop
  robot.halt();
  Serial.println("Calibration finished.");
  delay(1000);
}

void wallFollowing() {
  wallDist = sonar.readDist();
  
  // Calculate error from desired wall distance
  double error = wallDist - WALL_DISTANCE;
  
  // Get PD controller output
  double pdOutput = pd_obs.update(error, 0);
  
  // Set motor speeds using motion primitives
  if (pdOutput > 0) {
    // Need to turn right
    robot.turn_right_and_forward(0.1, WALL_SPEED, pdOutput/100.0);
  } else if (pdOutput < 0) {
    // Need to turn left
    robot.turn_left_and_forward(0.1, WALL_SPEED, -pdOutput/100.0);
  } else {
    // Go straight
    robot.forward(0.1, WALL_SPEED);
  }
}

void detectBlackLine() {
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