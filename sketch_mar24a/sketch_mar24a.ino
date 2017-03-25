//define motor driver pins
#define dir_1 10
#define pwm_1 11
#define dir_2 12
#define pwm_2 13

struct location{
  float x;
  float y;
  bool dropOff;
};

bool finished = false;

location position;
location modules[] = {};

location target;
int speed;

void setup() {
  pinMode(pwm_1,OUTPUT);
  pinMode(dir_1,OUTPUT);
  pinMode(pwm_2,OUTPUT);
  pinMode(dir_2,OUTPUT);
  selectTarget();
}

void loop() {
  if (finished == true){
    return;
  }
  moveToTarget();
  if (checkObstacle() == true){
    avoidObstacle();
  }
  if (atTarget() == true){
    if (target.dropOff == true){
      reverseIntake();
      selectTarget();
      }  
  }  
}

bool checkObstacle(){
  
}

void moveToTarget(){
  
}

void selectTarget(){
  
}

void avoidObstacle(){
  
}

bool atTarget(){
  
}

void reverseIntake(){
  
}

void moveForward(){
  int pwm_value = 50; // speed of the motor from 0 to 255
  digitalWrite(dir_1,LOW);
  digitalWrite(dir_2,HIGH);
  analogWrite(pwm_1,pwm_value);
  analogWrite(pwm_2,pwm_value);
}

void face(location target){
  angle = atan2(target.x - position.x, target.y - position.y);
  rotate(angle)
}

void rotate(float angle){
  digitalWrite(dir_1,LOW);
  digitalWrite(dir_2,LOW);
  analogWrite(pwm_1,pwm_value);
  analogWrite(pwm_2,pwm_value);
}


