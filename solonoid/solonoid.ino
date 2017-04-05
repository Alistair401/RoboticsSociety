#define sol_enable 29

void setup() {
  // put your setup code here, to run once:
  pinMode(sol_enable, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 digitalWrite(sol_enable, LOW);
 delay(5000);
 launchRocket();
 delay(5000);
}

void launchRocket(){
  digitalWrite(sol_enable, HIGH);
}
