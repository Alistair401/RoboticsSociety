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

int l_motor_rotation = 0;
int r_motor_rotation = 0;

struct location{
  float x;
  float y;
  bool dropOff;
};

bool finished = false;

location pos;
location modules[] = {};
location target;

int motor_speed;

void setup() {
  Serial.begin(9600);
  Serial.println("Robotics Team 2");
  Serial.println("SETUP...");
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

void reverseIntake(){
  
}

void moveForward(){
  digitalWrite(dir_r,LOW);
  digitalWrite(dir_l,HIGH);
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

void serialEvent(){
  char byteRead = Serial.read();
  Serial.print(byteRead);
}


