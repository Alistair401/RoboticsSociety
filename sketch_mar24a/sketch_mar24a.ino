// =========================================
// Code to run on the R2T2 robot for EuroBot
// Pins are for an Arduino Due, requires pozyx beacons and the counterpart code running on an Ardunio Uno
// STILL TODO:
// - control front intake (the pins for it have been defined already)
// - control the solonoid
// - logic for selecting the next location to go to, updating the current position (rolling totals are provided, just need to divide to get an average)
// - adjust rotation with compass heading (not essential, should be left for last)
// CHALLENGES:
// - aligning with the drop off zone
// - translating playing area locations to variables to store in the modules[] array (maybe modules is a bad name for it)

#include <elapsedMillis.h>

// define motor driver pins
// l = left, r = right
#define dir_r 10
#define pwm_r 11
#define dir_l 12
#define pwm_l 13

// define encoder pins
// l = left, r = right
#define encoder_r 6
#define encoder_l 8

// define IR sensor pins
// f = front side, l = left side, r = right side
#define ir_rec_l0 A1
#define ir_rec_l1 A2
#define ir_rec_f0 A3
#define ir_rec_f1 A4
#define ir_rec_r0 A5
#define ir_rec_r1 A0

// define solonoid pin
#define sol_enable 5

// define motor driver pins for front intake
#define forward_mot1 14
#define reverse_mot1 15
#define forward_mot2 4
#define reverse_mot2 3
#define enable_mot1 16
#define enable_mot2 2

struct location{
  int x;
  int y;
  bool dropOff;
};

location pos;
location modules[] = {};
location target;

struct message{
  char* string;
  size_t used;
  size_t capacity;
};

// Used to keep track of a rolling average to smooth out beacon values
// Uses a linked list of nodes each containing a location
struct node{
  location loc;
  node* next;
};

node* head;
int listCap = 4;
int listSize = 0;
int rollingXTotal = 0;
int rollingYTotal = 0;

message serialMessage;

elapsedMillis timeElapsed;

bool finished = false;

int motor_speed;

int l_motor_rotation = 0;
int r_motor_rotation = 0;

int timeout = 1000;


// =====
// SETUP
// =====
void setup() {
  // Initialize serial connection for debugging and communication with beacons
  Serial.begin(9600);
  Serial.println("Robotics Team 2");
  Serial.println("SETUP...");
  initMessage(&serialMessage,10);
  
  pinMode(pwm_r,OUTPUT);
  pinMode(dir_r,OUTPUT);
  pinMode(pwm_l,OUTPUT);
  pinMode(dir_l,OUTPUT);
  pinMode(ir_rec_l0,INPUT);
  pinMode(ir_rec_l1,INPUT);
  pinMode(ir_rec_f0,INPUT);
  pinMode(ir_rec_f1,INPUT);
  pinMode(ir_rec_r0,INPUT);
  pinMode(ir_rec_r1,INPUT);
  attachInterrupt(encoder_l,leftEncoderInterrupt,FALLING);
  attachInterrupt(encoder_r,rightEncoderInterrupt,FALLING);
  
  selectTarget();
}

// ===================
// MAIN LOOP AND LOGIC
// ===================
void loop() {
  if (finished == true){
    return;
  }
  moveToTarget();
  if (atTarget() == true){
    if (target.dropOff == true){
      reverseIntake();
      selectTarget();
      }  
  }
  if (timeElapsed >= 90000){
    launchRocket();
  }
}

bool checkObstacle(){
  // Checks the front sensors for obstacles
  // Not sure why we need the side sensors at all
  Serial.println("CHECKING SENSORS");
  if (analogRead(ir_rec_f0) < 800 || analogRead(ir_rec_f1) < 800){
    Serial.println("OBSTACLE DETECTED");
    return true;
  }
  return false;
}

void moveToTarget(){
  Serial.println("MOVING TO TARGET");
}

void selectTarget(){
  Serial.println("SELECTING TARGET");
}

void avoidObstacle(){
  Serial.println("AVOIDING OBSTACLE");
}

bool atTarget(){
  Serial.println("TARGET REACHED");
}


// ==========================
// MOVEMENT, MOTORS, ENCODERS
// ==========================
void reverseIntake(){
  
}

void moveForward(){
  digitalWrite(dir_r,LOW);
  digitalWrite(dir_l,HIGH);
  analogWrite(pwm_r,motor_speed);
  analogWrite(pwm_l,motor_speed);
}

void stopMotors(){
  analogWrite(pwm_r,motor_speed);
  analogWrite(pwm_l,motor_speed);
}

void face(){
  float angle = atan2(target.x - pos.x, target.y - pos.y);
  rotate(angle);
}

void rotate(float angle){
  // angle is in radians
  // 2000 pulses from the encoder is PI of rotation
  // To go clockwise, dir_l and dir_r are set HIGH
  int rotation_factor = 2000 / PI;
  bool clockwise = true;
  if (angle > PI){
    clockwise = false;
    angle = (2 * PI) - angle;
  }
  if (clockwise == true){
    digitalWrite(dir_r,HIGH);
    digitalWrite(dir_l,HIGH);
  } else {
    digitalWrite(dir_r,LOW);
    digitalWrite(dir_l,LOW);
  }
  int current_rot_l = l_motor_rotation;
  int current_rot_r = r_motor_rotation;
  analogWrite(pwm_r,motor_speed);
  analogWrite(pwm_l,motor_speed);
  
  Serial.println("ROTATING");
  Serial.print("ANGLE: ");
  Serial.print(angle);
  Serial.print(" | CLOCKWISE: ");
  Serial.println(clockwise);
  
  int i = 0;
  while (l_motor_rotation - current_rot_l < angle * rotation_factor){
    if (i % 20 == 0){
      Serial.print(".");
    }
    i++;
  }

  analogWrite(pwm_r,0);
  analogWrite(pwm_l,0);
}

void leftEncoderInterrupt(){
  l_motor_rotation++;
}

void rightEncoderInterrupt(){
  r_motor_rotation++;
}

// ==========================================
// USED FOR SERIAL COMMUNICATION WITH BEACONS
// ==========================================
void serialEvent(){
  char byteRead = Serial.read();
  if (byteRead == '\n'){
    addToMessage(&serialMessage,'\0');
    if (serialMessage.string[0] == 'S'){
      // Only message to start with S is "STOP"
      Serial.println("ROBOT DETECTED");
      detectedRobot();
    } else {
      // Only other messages should be coordinates, comma separated
      int xData,yData;
      sscanf(serialMessage.string,"%d,%d",&xData,&yData);
      addToLL((location){xData,yData,false});
    }
    freeMessage(&serialMessage);
    initMessage(&serialMessage,10);
  }
  else{
    addToMessage(&serialMessage,byteRead);
  }
}

void initMessage(message* m, size_t initialCapacity){
  m->string = (char*) malloc(initialCapacity*sizeof(char));
  memset(m->string,'\0',sizeof(m->string));
  m->capacity = initialCapacity;
  m->used = 0;
}

void addToMessage(message* m, char c){
  if (m->used == m->capacity){
    m->capacity *= 2;
    m->string = (char*) realloc(m->string,m->capacity * sizeof(char));
  }
  m->string[m->used] = c;
  m->used++;
}

void freeMessage(message *m){
  free(m->string);
  m->used = m->capacity = 0;
}

void detectedRobot(){
  int currentTime = timeElapsed;
  while (timeElapsed - currentTime < timeout){
    stopMotors();
  }
}

// ====
// MISC
// ====
void launchRocket(){
  
}

int distance(location* l1, location* l2){
  int yDiff = l1->y - l2->y;
  int xDiff = l1->x - l2->x;
  int dist  = (int)sqrt((yDiff * yDiff)+(xDiff * xDiff));
  return dist;
}

void addToLL(location l){
  if (listSize == listCap){
    rollingXTotal -= head->loc.x;
    rollingYTotal -= head->loc.y;
    free(head);
    head = head->next;
    listSize--;
  }
  if (head == NULL){
      head = (node*)malloc(sizeof(node));
      head->loc = l;
      head->next = NULL;
      } else {
      node* currentNode = head;
      while(currentNode->next != NULL){
        currentNode = currentNode->next;
      }
      currentNode->next = (node*)malloc(sizeof(node));
      currentNode->next->loc = l;
      currentNode->next->next = NULL;
    }
    rollingXTotal += l.x;
    rollingYTotal += l.y;
    listSize++;
}

