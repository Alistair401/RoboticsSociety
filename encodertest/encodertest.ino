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

int motor_speed = 100;

int l_motor_rotation = 0;
int r_motor_rotation = 0;

void setup() {
    Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(pwm_r,OUTPUT);
  pinMode(dir_r,OUTPUT);
  pinMode(pwm_l,OUTPUT);
  pinMode(dir_l,OUTPUT);
  attachInterrupt(encoder_l,leftEncoderInterrupt,FALLING);
  attachInterrupt(encoder_r,rightEncoderInterrupt,FALLING);

  digitalWrite(dir_r,LOW);
  digitalWrite(dir_l,HIGH);
  analogWrite(pwm_r,motor_speed);
  analogWrite(pwm_l,motor_speed);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  Serial.print("Left: ");
  Serial.println(l_motor_rotation);
  Serial.print("Right: ");
  Serial.println(r_motor_rotation);
}

void leftEncoderInterrupt(){
  l_motor_rotation++;
}

void rightEncoderInterrupt(){
  r_motor_rotation++;
}
