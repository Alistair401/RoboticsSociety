#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Magnetometer Test"); Serial.println("");
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}
void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  float Pi = 3.14159;
  // Calculate the angle of the vector y,x
  float rads = atan2(event.magnetic.y, event.magnetic.x);

  if(rads < 0){
    rads = rads + 2*PI;
  }
  
  float heading = (rads * 180) / PI;

  Serial.print(event.magnetic.x);
  Serial.print(", ");
  Serial.print(event.magnetic.y);
  Serial.print(", ");
  Serial.print(event.magnetic.z);
  Serial.print("  Compass Heading: ");
  Serial.print(heading);
  Serial.print(" degrees or ");
  Serial.print(rads);
  Serial.println(" radians.");
  delay(500);
}
