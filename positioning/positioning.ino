// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
/**
  The Pozyx ready to localize tutorial (c) Pozyx Labs
  
  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Arduino
  
  This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
  of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
  parameters and upload this sketch. Watch the coordinates change as you move your device around!
*/
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

//The ID of the big bad guy we are supposed to avoid
uint16_t the_big_bad = 0x6646;                            // set this to the ID of the remote device
bool avoid_the_big_bad = true;                           // For testing purposes

uint8_t num_anchors = 3;                                    // the number of anchors
uint16_t anchors[3] = {0x6645, 0x6600, 0x661E};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[3] = {0, 1250, 0};              // anchor x-coorindates in mm (-> that way)
int32_t anchors_y[3] = {0, 595, 1190};              // anchor y-coordinates in mm (v that way)
int32_t anchors_z[3] = {0, 0, 0};              // anchor z-coordinates in mm (height)


uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use
uint8_t dimension = POZYX_3D;                           // positioning dimension
int32_t height = 200;                                  // height of device, required in 2.5D positioning


////////////////////////////////////////////////

void setup(){
  Serial.begin(9600);
  
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.print(F("ERROR: Unable to connect to POZYX shield\n"));
    delay(100);
    abort();
  }
  
  // clear all previous devices in the device list
  Pozyx.clearDevices();
  // sets the anchor manually
  setAnchorsManual();

  //printCalibrationResult();
  //delay(2000);

  //Serial.print(F("Starting positioning:\n"));
}

void loop(){
  
  coordinates_t position;

  int status;  
  
  if(avoid_the_big_bad){
    trackTheBigBad();
  }
  
  status = Pozyx.doPositioning(&position, dimension, height, algorithm);
  
  if (status == POZYX_SUCCESS){
    // prints out the result
    printCoordinates(position);
  }else{
    // prints out the error code
    printErrorCode("positioning");
  }
}

void trackTheBigBad(){
  device_range_t big_bad_distance;
  int status;
  
  status = Pozyx.doRanging(the_big_bad, &big_bad_distance);
  
  if (status == POZYX_SUCCESS){
    if(big_bad_distance.distance < 1000){
      //Fucking panic the big bad is out to get us
      Serial.print("THE BIG BAD\n");
    }
  }else{
    // Assume the big bad has been destroyed
    printErrorCode("opponent not found!");
  }
}

// prints the coordinates for arduino/serial monitor
void printCoordinates(coordinates_t coor){
  Serial.print(coor.x);
  Serial.print(",");
  Serial.print(coor.y);
  Serial.print(",");
  Serial.print(coor.z);
  Serial.print("\n");
}

// error printing function for debugging
void printErrorCode(String operation){
  uint8_t error_code;

  Pozyx.getErrorCode(&error_code);
  Serial.print("ERROR ");
  Serial.print(operation);
  Serial.print(", local error code: 0x");
  Serial.print(error_code, HEX);
  Serial.print("\n");
}

// print out the anchor coordinates
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size);
  Serial.print("list size: ");
  Serial.print(status*list_size);
  Serial.print("\n");
  if(list_size == 0){
    printErrorCode("configuration");
    return;
  }
  
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids, list_size);
  
  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);
  
  coordinates_t anchor_coor;
  for(int i = 0; i < list_size; i++)
  {
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");    
    Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.print(anchor_coor.z);
    Serial.print("\n");
  }    
}

// function to manually set the anchor coordinates
void setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1; 
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = anchors_z[i];
    Pozyx.addDevice(anchor);
 }

