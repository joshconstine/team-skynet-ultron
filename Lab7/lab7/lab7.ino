#include <Pololu3piPlus32U4.h>
#include "printOLED.h"
#include "odometry.h"
#include "PIDcontroller.h"

using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;

// Odometry Parameters
#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define DEAD_RECKONING false

// PID for Angle Correction
#define minOutputAng -100
#define maxOutputAng 100
#define kpAng 1.5
#define kdAng 0.5
#define kiAng 0.01
#define clamp_iAng 50
#define base_speedAng 50

// PID for Velocity Control
#define minOutputVel -100
#define maxOutputVel 100
#define kpVel 2.0
#define kdVel 0.3
#define kiVel 0.02
#define clamp_iVel 50
#define base_speedVel 80

// Goal Position
const float goal_x = 1.0;
const float goal_y = 1.0;
const float goal_theta = 3.14 / 4;

// Odometry Variables
int16_t deltaL = 0;
int16_t deltaR = 0;
int16_t leftSpeed = 0;
int16_t rightSpeed = 0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 0.0, y = 0.0, theta = 0.0;

// PID Output Variables
double PIDout_theta, PIDout_distance;
double angle_to_goal, actual_angle;
double dist_to_goal = 0.0;
#define goal_threshold 0.5 // Stop when within 0.5 cm of the goal

// Initialize Odometry and PID Controllers
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
PIDcontroller pid_angle(kpAng, kiAng, kdAng, minOutputAng, maxOutputAng, clamp_iAng);
PIDcontroller pid_velocity(kpVel, kiVel, kdVel, minOutputVel, maxOutputVel, clamp_iVel);

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Read encoder data
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    encCountsLeft += deltaL;
    encCountsRight += deltaR;

    // Update odometry
    odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);

    // Compute angle to goal
    angle_to_goal = atan2(goal_y - y, goal_x - x);
    actual_angle = theta;

    // Normalize angle error to [-π, π]
    double error_theta = angle_to_goal - actual_angle;
    error_theta = atan2(sin(error_theta), cos(error_theta)); // Normalize

    // Compute distance to goal
    dist_to_goal = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));

    // Stop if close enough to the goal
    if (dist_to_goal < goal_threshold) {
        motors.setSpeeds(0, 0);
        Serial.println("Goal Reached!");
        return;
    }

    Serial.print("actual angle: ");
    Serial.print(actual_angle);
    Serial.print(" angle to goal: ");
    Serial.print(angle_to_goal);
    Serial.print(" dist to goal: ");
        Serial.print(dist_to_goal);
        Serial.println();
    // PID Control for Angle Correction
    PIDout_theta = pid_angle.update(actual_angle, angle_to_goal);

    // PID Control for Velocity
    PIDout_distance = pid_velocity.update(0, dist_to_goal);

    // Compute motor speeds
    leftSpeed = PIDout_distance - PIDout_theta;
    rightSpeed = PIDout_distance + PIDout_theta;

  Serial.print("left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" Right Speed: ");
    Serial.print(rightSpeed);
    // Set motor speeds
    motors.setSpeeds(leftSpeed, rightSpeed);

    // Debugging Output
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Theta: ");
    Serial.print(theta);
    Serial.print(" Error Theta: ");
    Serial.print(error_theta);
    Serial.print(" PID Out Theta: ");
    Serial.print(PIDout_theta);
    Serial.print(" PID Out Distance: ");
    Serial.println(PIDout_distance);

    delay(100);
}
