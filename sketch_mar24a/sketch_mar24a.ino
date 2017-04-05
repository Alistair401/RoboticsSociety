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
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);


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
#define sol_enable 29

// define motor driver pins for front intake
#define forward_mot1 14
#define reverse_mot1 15
#define forward_mot2 5
#define reverse_mot2 3
#define enable_mot1 16
#define enable_mot2 2

#define gyro_data 20 
#define gyro_clock 21

bool intake_intaking = true;

struct location{
  int x;
  int y;
  bool pickUp;
};

location pos;

bool returning = false;
int current_path_node = 0;
int path_size = 7;

location path[] = {
  //The edge of the ramp leading out the spaceship
   (location){360, 170, false},
   (location){890, 170, false},
   //The first cylinder in our path
   (location){950,200,true},
   (location){1150, 0, true},
   (location){1150, 0, true},
   (location){1150, 0, true},
   (location){1150, 0, true},
};

int current_module = 0;
int num_modules = 1;

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

int motor_speed = 100;

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

  if(!mag.begin()){
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
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
  pinMode(enable_mot1,OUTPUT);
  pinMode(enable_mot2,OUTPUT);
  pinMode(forward_mot1,OUTPUT);
  pinMode(forward_mot2,OUTPUT);
  pinMode(reverse_mot1,OUTPUT);
  pinMode(reverse_mot2,OUTPUT);
  digitalWrite(enable_mot1,HIGH);
  digitalWrite(enable_mot2,HIGH);

  setIntakePull();
  attachInterrupt(encoder_l,leftEncoderInterrupt,FALLING);
  attachInterrupt(encoder_r,rightEncoderInterrupt,FALLING);
  selectTarget();
}

// ===================
// MAIN LOOP AND LOGIC
// ===================
void loop() {
   rotate(PI/2);
   moveForwards();
   delay(7000);
   moveBackwards();
   delay(7000);
   dropOff();
   delay(20000);
   return;

  if (finished == true){
    return;
  }
  
  moveToTarget();

  if (atTarget() == true){
    stopMotors();

    if (target.pickUp == true){
      reverseIntake();
      delay(10);
      moveBackwards();
      delay(100);
      reverseIntake();
    }  

    selectTarget();
  }
  
  if (timeElapsed >= 90000){
    finish();
  }

}

void finish(){
  stopMotors();
  launchRocket();
    
  finished = true;
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

  Serial.println(getAngle(pos, target));
  if(abs(getAngle(pos, target)) > 0.1){
    face(target);
  }else{
    moveForwards();
  }
}

void selectTarget(){
  Serial.println("SELECTING TARGET");

  if(path[current_path_node].pickUp==true){
    path[current_path_node].pickUp = false;
    returning = true;
  }

  if(returning){
    if(current_path_node == 0){
      dropOff();
      returning = false;
    }else{
      current_path_node -= 1;
      target = path[current_path_node];
    }
  } else {
    if(current_path_node < path_size){
      current_path_node += 1;
    }

    target = path[current_path_node];
  }
}

void avoidObstacle(){
  Serial.println("AVOIDING OBSTACLE lol no we're not stop");
}

bool atTarget(){
  if(distance(&pos, &target) < 5000){
    Serial.println("TARGET REACHED");
    return true;
  }

  return false;
}

void dropOff(){
  setIntakePush();
  delay(5000);
  moveBackwards();
  delay(1000);
  stopMotors();
  setIntakePull();
}


// ==========================
// MOVEMENT, MOTORS, ENCODERS
// ==========================
void reverseIntake(){
    //Reverse the motor
    if(intake_intaking){
      setIntakePush();
    }else{
      setIntakePull();
    }
}

void setIntakePush(){
  intake_intaking = false;
  digitalWrite(forward_mot1,false);
  digitalWrite(forward_mot2,false);
  digitalWrite(reverse_mot1,true);
  digitalWrite(reverse_mot2,true);
}

void setIntakePull(){
  intake_intaking = true;
  digitalWrite(forward_mot1,true);
  digitalWrite(forward_mot2,true);
  digitalWrite(reverse_mot1,false);
  digitalWrite(reverse_mot2,false);
}

void moveForwards(){
  digitalWrite(dir_r,HIGH);
  digitalWrite(dir_l,HIGH);
  analogWrite(pwm_r,motor_speed);
  analogWrite(pwm_l,motor_speed);
}

void moveBackwards(){
  digitalWrite(dir_r,LOW);
  digitalWrite(dir_l,LOW);
  analogWrite(pwm_r,motor_speed);
  analogWrite(pwm_l,motor_speed);
}

void stopMotors(){
  analogWrite(pwm_r,0);
  analogWrite(pwm_l,0);
}

void face(location target){
  float angle = getAngle(pos,target);
  rotate(angle);
}

float getAngle(location x, location y){
  return atan2(y.x - x.x, y.y - x.y);
}

/*
//Rotate using gyroscope
void rotate(float angle){
  
  sensors_event_t event; 
  mag.getEvent(&event);

  float startrot = atan2(event.magnetic.y,event.magnetic.x);

  bool clockwise = true;
  
  if (angle < 0){
    clockwise = false;
    angle = -1* angle;
  }
  
  if (clockwise == true){
    digitalWrite(dir_r,LOW);
    digitalWrite(dir_l,HIGH);
  } else {
    digitalWrite(dir_r,HIGH);
    digitalWrite(dir_l,LOW);
  }

  float rot = startrot;

  analogWrite(pwm_r,motor_speed);
  analogWrite(pwm_l,motor_speed);
  
  Serial.println("ROTATING");
  while (rot - startrot < angle){
    mag.getEvent(&event);
    rot = atan2(event.magnetic.y,event.magnetic.x);
    float delta = rot - startrot;
    Serial.println(rot);
    
    if(delta >= angle){
      break;
    }
  }

  stopMotors();
}
*/

//Rotate using encoders
void rotate(float angle){
  // angle is in radians
  // 2000 pulses from the encoder is PI of rotation
  // To go clockwise, dir_l and dir_r are set HIGH and LOW, but not necessarily in that order
  int rotation_factor = 2000/PI;
  bool clockwise = true;
  Serial.println("ANGLE");
  Serial.println(angle);
  
  if (angle < 0){
    clockwise = false;
    angle = fmod(PI + -1*angle, 2*PI);
  }

  if (angle > PI){
    clockwise = false;
    angle = fmod(angle,PI);
  }
  
  if (clockwise == true){
    digitalWrite(dir_r,LOW);
    digitalWrite(dir_l,HIGH);
  } else {
    digitalWrite(dir_r,LOW);
    digitalWrite(dir_l,HIGH);
  }
  
  l_motor_rotation = 0;
  r_motor_rotation = 0;
  int current_rot_l = 0;
  int current_rot_r = 0;
  
  analogWrite(pwm_r,motor_speed);
  analogWrite(pwm_l,motor_speed);
  
  Serial.println("ROTATING");
  Serial.print("ANGLE: ");
  Serial.print(angle);
  Serial.print(" | CLOCKWISE: ");
  Serial.println(clockwise);
  
  while (l_motor_rotation - current_rot_l < angle * rotation_factor){
  }

  analogWrite(pwm_r,0);
  analogWrite(pwm_l,0);
}

void leftEncoderInterrupt(){
  l_motor_rotation++;
  Serial.println(l_motor_rotation);
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
  digitalWrite(sol_enable, HIGH);
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

  pos.x = rollingXTotal/(float)listSize;
  pos.y = rollingYTotal/(float)listSize;
}

