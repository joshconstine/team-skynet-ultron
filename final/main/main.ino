#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4LineSensors.h>
#include <Pololu3piPlus32U4Motors.h>
#include <Pololu3piPlus32U4OLED.h>
#include <Pololu3piPlus32U4Buzzer.h>
#include <Pololu3piPlus32U4BumpSensors.h>
#include "sonar.h"
#include "PDcontroller.h"

using namespace Pololu3piPlus32U4;

enum RobotState {
  WANDER,
  CELEBRATION,
  RETURN_HOME
};

RobotState currentState = WANDER;
int trashCount = 0;
bool atHome = false;
bool isOnBlack = false;

#define MAX_TRASH 3
#define CELEBRATION_DURATION 3000
#define SPIN_SPEED 100
#define MOVE_SPEED 100
#define TURN_DELAY 500
#define MOVE_DELAY 1000

// Maze dimensions: 80x180 cm → 8x18 grid (10 cm cells)
const int MAZE_WIDTH = 8;
const int MAZE_HEIGHT = 18;

bool visited[MAZE_HEIGHT][MAZE_WIDTH] = {false};
int robotX = 0;
int robotY = 17; // Start in bottom-left corner

enum Direction { NORTH, EAST, SOUTH, WEST };
Direction heading = NORTH;

struct Position {
  int x, y;
};
Position stack[MAZE_WIDTH * MAZE_HEIGHT];
int stackSize = 0;

uint16_t lineDetectionValues[5];

LineSensors lineSensors;
Motors motors;
OLED display;
Buzzer buzzer;
BumpSensors bumpSensors;
Sonar sonar(4);

#define kp_obs 6
#define kd_obs 3
#define minOutput -100
#define maxOutput 100
PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);

// Stack functions
void push(Position pos) {
  if (stackSize < MAZE_WIDTH * MAZE_HEIGHT)
    stack[stackSize++] = pos;
}

Position pop() {
  if (stackSize > 0)
    return stack[--stackSize];
  return {robotX, robotY};
}

bool isStackEmpty() {
  return stackSize == 0;
}

// Direction handling
void turnLeft90() {
  motors.setSpeeds(-MOVE_SPEED, MOVE_SPEED);
  delay(TURN_DELAY);
  motors.setSpeeds(0, 0);
  heading = (Direction)((heading + 3) % 4);
}

void turnRight90() {
  motors.setSpeeds(MOVE_SPEED, -MOVE_SPEED);
  delay(TURN_DELAY);
  motors.setSpeeds(0, 0);
  heading = (Direction)((heading + 1) % 4);
}

void faceDirection(Direction target) {
  while (heading != target) {
    turnRight90();
  }
}

// Movement
void moveForwardOneCell() {
  motors.setSpeeds(MOVE_SPEED, MOVE_SPEED);
  delay(MOVE_DELAY);
  motors.setSpeeds(0, 0);

  switch (heading) {
    case NORTH: robotY--; break;
    case SOUTH: robotY++; break;
    case EAST:  robotX++; break;
    case WEST:  robotX--; break;
  }
}

// Obstacle detection
bool wallInFront() {
  double dist = sonar.readDist();
  return dist < 7.0; // consider wall if less than 7cm
}

// Coordinate validity
bool isValid(int x, int y) {
  return x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT;
}

// Detect black line (e.g., trash)
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

// Setup
void setup() {
  Serial.begin(9600);
  bumpSensors.calibrate();
  // lineSensors.initFiveSensors();

  display.clear();
  display.print("Trash: 0");
  delay(1000);
}

// Main loop
void loop() {
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

// DFS traversal
void wanderState() {
  detectBlackLine();

  if (trashCount >= MAX_TRASH) {
    currentState = RETURN_HOME;
    return;
  }

  if (isOnBlack) {
    currentState = CELEBRATION;
    return;
  }

  if (isStackEmpty()) {
    push({robotX, robotY});
  }

  Position current = pop();
  visited[current.y][current.x] = true;

  const int dx[4] = {0, 1, 0, -1};  // N, E, S, W
  const int dy[4] = {-1, 0, 1, 0};

  for (int dir = 0; dir < 4; dir++) {
    int nx = current.x + dx[dir];
    int ny = current.y + dy[dir];

    if (isValid(nx, ny) && !visited[ny][nx]) {
      faceDirection((Direction)dir);
      if (!wallInFront()) {
        moveForwardOneCell();
        push({nx, ny});
        return;
      }
    }
  }

  // Dead end, backtrack
  if (!isStackEmpty()) {
    Position prev = pop();
    int dx = prev.x - robotX;
    int dy = prev.y - robotY;
    if (dx == 1) faceDirection(EAST);
    else if (dx == -1) faceDirection(WEST);
    else if (dy == 1) faceDirection(SOUTH);
    else if (dy == -1) faceDirection(NORTH);
    moveForwardOneCell();
    push(prev);
  }
}

// Celebration
void celebrationState() {
  motors.setSpeeds(0, 0);
  motors.setSpeeds(SPIN_SPEED, -SPIN_SPEED);
  delay(CELEBRATION_DURATION);
  motors.setSpeeds(0, 0);

  display.clear();
  trashCount++;
  display.print("Trash: ");
  display.print(trashCount);

  isOnBlack = false;
  currentState = WANDER;
}

// Return home (basic placeholder)
void returnHomeState() {
  motors.setSpeeds(0, 0);
  display.clear();
  display.print("Returning...");
  delay(3000);
  currentState = WANDER;
}
