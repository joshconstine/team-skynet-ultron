#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4LineSensors.h>
#include <Pololu3piPlus32U4Motors.h>
#include <Pololu3piPlus32U4OLED.h>
#include <Pololu3piPlus32U4Buzzer.h>
#include <Pololu3piPlus32U4BumpSensors.h>
#include <Servo.h>

#include "sonar.h"
#include "PDcontroller.h"

using namespace Pololu3piPlus32U4;

// Robot states
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
#define CELEBRATION_DURATION 3200
#define SPIN_SPEED 100
#define MOVE_SPEED 100
#define TURN_DELAY 900
#define MOVE_DELAY 2000

// Maze dimensions: 80x180 cm → 8x18 grid (10 cm cells)
const int MAZE_WIDTH = 8;
const int MAZE_HEIGHT = 18;

// Track visited cells and trash locations
bool visited[MAZE_HEIGHT][MAZE_WIDTH] = {false};
bool trashFound[MAZE_HEIGHT][MAZE_WIDTH] = {false};
int robotX = 0;
int robotY = 17; // Start in bottom-left corner

// Store trash positions for return path
struct Position {
  int x, y;
};
Position trashPositions[MAX_TRASH];
int trashPositionsCount = 0;

enum Direction { NORTH, EAST, SOUTH, WEST };
Direction heading = NORTH;

Position stack[MAZE_WIDTH * MAZE_HEIGHT];
int stackSize = 0;

uint16_t lineDetectionValues[5];

LineSensors lineSensors;
Motors motors;
OLED display;
Servo servo;
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
  Serial.print("Sonar Distance: ");
  Serial.print(dist);
  Serial.println(" cm");
  return dist < 11.0; // consider wall if less than 10cm
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

// Print the current map state
void printMap() {
  Serial.println("\n=== Current Map State ===");
  Serial.println("Legend: . = unvisited, X = wall, V = visited, T = trash");
  Serial.println("------------------------");
  
  for (int y = 0; y < MAZE_HEIGHT; y++) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      if (x == robotX && y == robotY) {
        Serial.print("R"); // Robot position
      } else if (trashFound[y][x]) {
        Serial.print("T"); // Trash location
      } else if (visited[y][x]) {
        Serial.print("V"); // Visited cell
      } else {
        Serial.print("."); // Unvisited cell
      }
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("------------------------");
}

// Setup
void setup() {
  Serial.begin(9600);
  bumpSensors.calibrate();
  // lineSensors.initFiveSensors();
  servo.attach(5);

  servo.write(90);

  display.clear();
  display.print("Trash: 0");
  delay(1000);
}

// Main loop
void loop() {
  // Print current state and map
  Serial.print("\nCurrent State: ");
  switch (currentState) {
    case WANDER: Serial.println("WANDER"); break;
    case CELEBRATION: Serial.println("CELEBRATION"); break;
    case RETURN_HOME: Serial.println("RETURN_HOME"); break;
  }
  
  Serial.print("Robot Position: (");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.print("), Heading: ");
  switch (heading) {
    case NORTH: Serial.println("NORTH"); break;
    case EAST: Serial.println("EAST"); break;
    case SOUTH: Serial.println("SOUTH"); break;
    case WEST: Serial.println("WEST"); break;
  }
  
  printMap();
  
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
  
  delay(1000); // Add delay to make serial output readable
}

// DFS traversal
void wanderState() {
  detectBlackLine();

  if (trashCount >= MAX_TRASH) {
    currentState = RETURN_HOME;
    return;
  }

  if (isOnBlack && !trashFound[robotY][robotX]) {
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

    if (isValid(nx, ny) && !visited[ny][nx] && !trashFound[ny][nx]) {
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
  // Record trash position
  if (trashPositionsCount < MAX_TRASH) {
    trashPositions[trashPositionsCount] = {robotX, robotY};
    trashPositionsCount++;
  }
  
  // Mark this position as having trash
  trashFound[robotY][robotX] = true;
  
  // Stop and celebrate
  motors.setSpeeds(0, 0);
  motors.setSpeeds(SPIN_SPEED, -SPIN_SPEED);
  delay(CELEBRATION_DURATION);
  motors.setSpeeds(0, 0);

  // Update display
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
