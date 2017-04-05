#include <elapsedMillis.h>
#define enable_mot1 16
#define forward_mot1 14
#define reverse_mot1 15

#define enable_mot2 2
#define forward_mot2 5
#define reverse_mot2 3

elapsedMillis timeElapsed;

void setup() {
  // put your setup code here, to run once:
  pinMode(enable_mot1,OUTPUT);
  pinMode(enable_mot2,OUTPUT);
  
  pinMode(forward_mot1,OUTPUT);
  pinMode(forward_mot2,OUTPUT);
  
  pinMode(reverse_mot1,OUTPUT);
  pinMode(reverse_mot2,OUTPUT);
  
  digitalWrite(enable_mot1,HIGH);
  digitalWrite(enable_mot2,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(timeElapsed%10000 > 5000){
    digitalWrite(forward_mot1,false);
    digitalWrite(forward_mot2,false);
    
    digitalWrite(reverse_mot1,true);
    digitalWrite(reverse_mot2,true);
  }else{
    digitalWrite(forward_mot1,true);
    digitalWrite(forward_mot2,true);
    
    digitalWrite(reverse_mot1,false);
    digitalWrite(reverse_mot2,false);
  }
}
