#include "vector2.h"

// Put key coordinates here

const vector2 NORTH = {0,1};
vector2 target = {0,0};
vector2 currentPos = {0,0};
float currentRot = 0; // Measured from north

void setup() {
  // Setup code
}

void loop() {
  // Main loop

  // Update path to target
  vector2 targetDirection = getDirection(currentPos,target);
  float targetRot = getAngle(NORTH,targetDirection);
  rotate(180 + targetRot - currentRot);

  // Avoid collisions OR carry out action if at target
}

// Returns a directional vector from vector v1 to v2
vector2 getDirection(vector2 v1, vector2 v2){
  return (vector2){v2.x-v1.x,v2.y-v1.y};
}

// Returns the angle from v1 to v2
float getAngle(vector2 v1, vector2 v2){
  float cosTheta = dotProduct(v1,v2) / (magnitude(v1)*magnitude(v2));
  return acos(cosTheta);
}

// Returns the dot product of v1 and v2
float dotProduct(vector2 v1, vector2 v2){
  return (v1.x * v2.x) + (v1.y + v2.y);
}

// Returns the magnitude of v
float magnitude(vector2 v){
  return sqrt(pow(v.x,2) + pow(v.y,2));
}

// Rotate the robot theta degrees clockwise
void rotate(float theta){
  // TODO
}

// Move the robot forward x units
void moveForward(float x){
  // TODO
}

