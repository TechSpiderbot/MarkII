#include <Wire.h>
#include <RGBLed.h>
/* https://github.com/wilmouths/RGBLed
*/
//#define DEBUG
#define COMPASS_ADDR 0x08 // hex 0x10 address for nucleo

bool I2C_error = false;

#define DATA_SIZE 2
int nucleoData[DATA_SIZE];
int LEG3_DATA[DATA_SIZE];
int LEG4_DATA[DATA_SIZE];
// RGB LED CONFIG
//            RED, GREEN, BLUE
int LEG4_RGB[3] = {11,10,9};
int LEG3_RGB[3] = {6,5,3};

RGBLed led4(LEG4_RGB[0], LEG4_RGB[1], LEG4_RGB[2],COMMON_CATHODE);
RGBLed led3(LEG3_RGB[0], LEG3_RGB[1], LEG3_RGB[2],COMMON_CATHODE);

// SENSORS CONFIG
int LEG_calb[4];
int LEG_analogPressure[4];
int LEG_bPSwitch[4];

// LEG4 SENSORS CONFIG
#define LEG4_PressurePin A3
#define LEG4_SwitchPin 4

//LEG3 SENSORS CONFIG
#define LEG3_PressurePin A2
#define LEG3_SwitchPin 7

//LEG2 SENSORS CONFIG
#define LEG2_PressurePin A1
#define LEG2_SwitchPin 8

//LEG1 SENSORS CONFIG
#define LEG1_PressurePin A0
#define LEG1_SwitchPin 12

#define offset 200
// Define counter to count bytes in response
int bcount = 0;

// reset arduino when does't get an ack
void(* resetFunc) (void) = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(LEG4_SwitchPin,INPUT);
  pinMode(LEG3_SwitchPin,INPUT);
  pinMode(LEG2_SwitchPin,INPUT);
  pinMode(LEG1_SwitchPin,INPUT);
  
  LEG_calb[3] = analogRead(LEG4_PressurePin);
  LEG_calb[2] = analogRead(LEG3_PressurePin);
  LEG_calb[1] = analogRead(LEG2_PressurePin);
  LEG_calb[0] = analogRead(LEG1_PressurePin);
 
  Wire.begin(COMPASS_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

//-----LIST---------------*
void checkData();
void SetColour();
void SerialMonitor();

//-----MAIN---------------*
void loop() {
  // Read sensor data
  LEG_bPSwitch[3] = digitalRead(LEG4_SwitchPin);
  LEG_bPSwitch[2] = digitalRead(LEG3_SwitchPin);
  LEG_bPSwitch[1] = digitalRead(LEG2_SwitchPin);
  LEG_bPSwitch[0] = digitalRead(LEG1_SwitchPin);

  LEG_analogPressure[0] = analogRead(LEG1_PressurePin) - LEG_calb[0];
  LEG_analogPressure[1] = analogRead(LEG2_PressurePin) - LEG_calb[1];
  LEG_analogPressure[2] = analogRead(LEG3_PressurePin) - LEG_calb[2];
  LEG_analogPressure[3] = analogRead(LEG4_PressurePin) - LEG_calb[3];
  
  // if debug mode is true show data on monitor
  SerialMonitor();

  
  // if leg 4 touch object fade rgb
  // temp test 
  /*
  if(LEG4_analogPressure >= (LEG4_calb + offset) && LEG4_bPSwitch){
    led4.fadeIn(RGBLed::MAGENTA, 5, 100);
    led4.fadeOut(RGBLed::MAGENTA, 5, 100);
  }
  else{ led4.off(); }

  // if leg 3 touch object fade rgb
  if(LEG3_analogPressure >= (LEG3_calb + offset) && LEG3_bPSwitch){
    led3.fadeIn(RGBLed::MAGENTA, 5, 100);
    led3.fadeOut(RGBLed::MAGENTA, 5, 100);
  }
  else{ led3.off(); }
  */
  
  delay(1000);
}

void CHECK_FLAGS(){
  if(I2C_error){
    I2C_error = false;
    resetFunc();   
  }
}
    
void checkData(){
  for(int i = 0; i <4; i++){
    if(LEG_bPSwitch[i] >= 1){
      LEG_bPSwitch[i] = 128;
    }
  }

  for(int i = 0; i <4; i++){
    if(LEG_analogPressure[i] <= 0){
      LEG_analogPressure[i] = 0;
    }
    else if(LEG_analogPressure[i] >= 252){
      LEG_analogPressure[i] = 252;
    }
  }
}

void requestEvent() {
// Define a byte to hold data
  int mLength = 9;

  checkData();
   
  char dataMessage[] = {255,
                        LEG_analogPressure[0],  // LEG1_analogPressure
                        LEG_analogPressure[1],  // LEG2_analogPressure
                        LEG_analogPressure[2],  // LEG3_analogPressure
                        LEG_analogPressure[3],  // LEG4_analogPressure
                        LEG_bPSwitch[0],        // LEG1_bPSwitch
                        LEG_bPSwitch[1],        // LEG2_bPSwitch
                        LEG_bPSwitch[2],        // LEG3_bPSwitch
                        LEG_bPSwitch[3]         // LEG4_bPSwitch
                       };
  
  // Send response back to Master
  Wire.write(dataMessage,mLength);
}

// receive data to controll RGB led status mode
void receiveEvent(){
  // clear nucleoData[]
  for(int i = 0; i < DATA_SIZE; i++){
      nucleoData[i] = 0;
  }

  // read incomming data
  while(0< Wire.available()){
    for(int i = 0; i < DATA_SIZE; i++){
      nucleoData[i] = Wire.read();

      if(nucleoData[0] == 3){
        LEG3_DATA[i] = nucleoData[i];
      }
      else if(nucleoData[0] == 4){
        LEG4_DATA[i] = nucleoData[i];
      }
    }
  }
  SetColour(); 
}

void SetColour(){
  /* 
  Serial.print("LEG3_DATA: \t");
  for(int i =0; i < DATA_SIZE; i++){
    Serial.print(LEG3_DATA[i]);
    Serial.print("\t");
  }
  Serial.println("");  
    
  Serial.print("LEG4_DATA: \t");
  for(int i =0; i < DATA_SIZE; i++){
    Serial.print(LEG4_DATA[i]);
    Serial.print("\t");
  }
  Serial.println("");
  */
  /*
     COLOURS:
    RED        1
    GREEN      2
    BLUE       4
    MAGENTA    8
    CYAN      16
    YELLOW    32
    WHITE     64
  */
  if( LEG3_DATA[0] == 3 && LEG3_DATA[1] == 1){ // blink
    led3.fadeOut(RGBLed::RED,5,100);
  }
  else {
    // LIGHT OFF
      led3.off();
  }
    
  if(LEG4_DATA[0] == 4 && LEG4_DATA[1] == 1){ // blink
    led4.fadeOut(RGBLed::RED,5,100);
  }
  else{
    led4.off();
  }
}

void SerialMonitor(){
#ifdef DEBUG
  checkData();
  // pressure sensor
  Serial.print("Pressure:\t");
  Serial.print(LEG_analogPressure[3]);
  Serial.print("\t");
  Serial.print(LEG_analogPressure[2]);
  Serial.print("\t");
  Serial.print(LEG_analogPressure[1]);
  Serial.print("\t");
  Serial.print(LEG_analogPressure[0]);
  Serial.print("\t");
  // switch sensor
  
  Serial.print("switch:\t");
  Serial.print(LEG_bPSwitch[3]);
  Serial.print("\t");
  Serial.print(LEG_bPSwitch[2]);
  Serial.print("\t");
  Serial.print(LEG_bPSwitch[1]);
  Serial.print("\t");
  Serial.println(LEG_bPSwitch[0]);
#endif
}
