
/*
 * Update 2.10
 * need new calibration when attached to the robot
 * 
 * >>>>int directionHeading(int heading){
 * 
 * #define DEBUG ENABLES SERIAL INTERFACE
 * #define CALIBRATION show calibration data else it shows direction data
 * #define I2C_ACTIVE ENABLES I2C COMUNICATION
 */ 

//-----INCLUDE-----------*
#include <MechaQMC5883.h>
#include <Wire.h>
#include <math.h>

//-----DEFINE-----------*
#define DEBUG
//#define CALIBRATION
#define I2C_ACTIVE

#define REVISION 2.10
#define DEG_PER_RAD (180.0/3.14159265358979)

// new connection instead of serial

#define SLAVE_ADDR 0x08 // hex 0x10 address for nucleo
#define DATA_SIZE 32

MechaQMC5883 qmc;

//-----VARIABLES----------*
//TEMP:
bool showCal = false;
//------------------------*
bool calibrated = false;
bool calibrationMode = false ;

// I2C
char nucleoData[DATA_SIZE] = "";
bool dataReceived = false;

int x_min;
int x_max;

int y_min;
int y_max;

int z_max;

int x_offset;
int y_offset;

float x_scale;
float y_scale;

String inputString = "";      // a String to hold incoming data
bool stringComplete = false;

int old_spriteNum = 8;
int spriteNum = 0;
//-----SETUP--------------*  
void setup() { 
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.print("REV: ");
    Serial.println(REVISION);
  #endif
  
    Wire.begin();
  #ifdef I2C_ACTIVE
    Wire.begin(SLAVE_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
  #endif
  
  qmc.init();
  inputString.reserve(200);
}

//-----LIST---------------*
void requestEvent(){ Wire.write(spriteNum);}

bool isCalibrated(){ return calibrated;}
bool isCalibrating(){ return calibrationMode;}

float readHeading();
void enterCalNorthMode();

void enterCalMode(); 
void calibrate(); 
void exitCalMode();

int directionHeading(int);

void showDirection(int);
void showCalibration();

//-----MAIN---------------*
void loop() {
  int x, y, z;
  int heading;
  

  // Sensor calibration--------------------------------------------------------
  //---------------------------------------------------------------------------
  //If we're not calibrated
  if(!isCalibrated()){
    
    //And we're not currently calibrating
    if(!isCalibrating()){
      #ifdef DEBUG
        Serial.println("Entering calibration mode");
        Serial.println(">>>> Press h when done....");
      #endif
      enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
    }
    else{
      //Must call every loop while calibrating to collect calibration data
      //This will automatically exit calibration
      //You can terminate calibration early by calling exitCalMode();
      calibrate(); 
    }
  }
  else if(isCalibrated()){ //isCalibrated(TRUE)
    if(showCal){
      showCal = false;
      showCalibration();
    } 
  }
  
  if(isCalibrated()){
    heading = (int)readHeading();   //reading
    spriteNum = directionHeading(heading);      //get direction
   
    // als new_value = 0 && old value = 7 OF new_value = 7 && old value = 0 >>> OK
    // >> ELSE
      // als new_value - old_value groter dan 1 >>> new_value + 1
      // als new_value - old_value kleiner dan  -1 >>> new value -1

    //if(spriteNum != old_spriteNum){
      
      if((!spriteNum ==0 && !old_spriteNum ==7)||(!spriteNum ==7 && !old_spriteNum ==0)){
        if((spriteNum - old_spriteNum) > 1){
          spriteNum = (old_spriteNum +1);
        }
      }
      else if((spriteNum - old_spriteNum) < -1){
        spriteNum = (old_spriteNum -1);
      }

    /* spriteNum = 0;    // N
     * spriteNum = 1;    // NE
     * spriteNum = 2;    // E
     * spriteNum = 3;    // SE
     * spriteNum = 4;    // S
     * spriteNum = 5;    // SW
     * spriteNum = 6;    // W
     * spriteNum = 7;    // NW
     */
      // show results
      showDirection(spriteNum);

      old_spriteNum =  spriteNum;
      delay(100);
    //}      
  }
}

// SERIAL EVENT----- --------------------------------------------------------
//---------------------------------------------------------------------------
void serialEvent(){
  #ifdef DEBUG
  while(Serial.available()){
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n'){
      stringComplete = true;
    }
  }
  #endif
}

// I2C EVENT----- -----------------------------------------------------------
//---------------------------------------------------------------------------
void receiveEvent(){
  
  while(0< Wire.available()){
    for(int i = 0; i < DATA_SIZE; i++){
      nucleoData[i] = Wire.read();
    }
  }
  dataReceived = true;
}

// READING DIRECTION --------------------------------------------------------
//---------------------------------------------------------------------------
float readHeading(){
  int x, y, z;
 
  qmc.read(&x,&y,&z);
  float xf = (float) x * 1.0f;
  float yf = (float) y * 1.0f;
  
  //Calculate the heading
  return (atan2(-y*y_scale, x*x_scale) * DEG_PER_RAD);
}

// CALIBRATION --------------------------------------------------------------
//---------------------------------------------------------------------------
void enterCalMode(){
  calibrationMode = true;
  //Starting values for calibration
  x_min = -20;
  x_max =  10;

  y_min = -20;
  y_max = -10;
 
  z_max = -10;
  calibrated = false;

  /*
   *  Mode : Mode_Standby / Mode_Continuous
   *
   *  ODR : ODR_10Hz / ODR_50Hz / ODR_100Hz / ODR_200Hz
   *  ouput data update rate
   *
   *  RNG : RNG_2G / RNG_8G
   *  magneticfield measurement range
   *
   *  OSR : OSR_512 / OSR_256 / OSR_128 / OSR_64
   *  over sampling rate
  */

  //Set to active mode, highest ODR & OSR for continous readings
  qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
}

void calibrate(){
  int x, y, z;
  bool changed = false;
  while(!changed){     // wait till robot is ready
    qmc.read(&x,&y,&z);
    
    if(x < x_min){
      x_min = x;
    }
    if(x > x_max){
      x_max = x;
    }
    if(y < y_min){
      y_min = y;
    }
    if(y > y_max){
      y_max = y;
    }
    if(z < z_max){
      z_max = z;
    } 
    
    serialEvent();

    #ifdef I2C_ACTIVE
      receiveEvent();
    #endif
    
    delay(10);
    if((stringComplete && inputString == "h\n") || (nucleoData[0] == 'h')){
      changed = true;
      inputString = "";
      stringComplete =  false;
      dataReceived = false;
    }
  }
   
  exitCalMode();
}

void exitCalMode(){
  //Calculate offsets
  x_offset = (x_min + x_max)/2;

  y_offset = (y_min + y_max)/2;

  x_scale = 1.0/(x_max - x_min);
  y_scale = 1.0/(y_max - y_min);
  
  calibrationMode = false;
  calibrated = true;
  showCal = true;
}

int directionHeading(int heading){
  String buff;
    
  if (heading < 0){ //If the heading is negative
    heading *= -1; //Negative the heading
  }
    #ifdef CALIBRATION
      Serial.print(heading); 
      Serial.print("\t");
    #endif
    
    int spriteNum = 0;

    if(heading < 10 && heading > 0 || heading >= 170){
      spriteNum = 0;
      buff = ("N");
    }
    else if(heading >= 150 && heading < 170 ){
      spriteNum = 1;
      buff = ("NE");
    }
    else if(heading >= 130 && heading < 150 ){
      spriteNum = 2;
      buff = ("E");
    }
    else if(heading >= 100 && heading < 130 ){
      spriteNum = 3;
      buff = ("SE");
    }
    else if(heading >= 80 && heading < 100 ){
      spriteNum = 4;
      buff = ("S");
    } 
    else if(heading >= 55 && heading < 80 ){
      spriteNum = 5;
      buff = ("SW");
    }
    else if(heading >= 30 && heading < 55 ){
      spriteNum = 6;
      buff = ("W");
    }
    else if(heading >= 10 && heading < 30 ){
      spriteNum = 7;
      buff = ("NW");
    }

    #ifdef CALIBRATION
      Serial.println(buff);
    #endif
    
  return spriteNum;
}
// SHOW EVENT----- ----------------------------------------------------------
//---------------------------------------------------------------------------
void showDirection(int d){
  String buff;
    /* spriteNum = 0;    // N
     * spriteNum = 1;    // NE
     * spriteNum = 2;    // E
     * spriteNum = 3;    // SE
     * spriteNum = 4;    // S
     * spriteNum = 5;    // SW
     * spriteNum = 6;    // W
     * spriteNum = 7;    // NW
     */
  
    
    if(d == 0){
      buff = ("N");  
    }
    else if(d == 1){
      buff = ("NE");
    }
    else if(d == 2){
      buff = ("E");  
    }
    else if(d == 3){
      buff = ("SE");  
    }
    else if(d == 4){
      buff = ("S");  
    }
    else if(d == 5){
      buff = ("SW");  
    }
    else if(d == 6){
      buff = ("W"); 
    }
    else if(d == 7){
      buff = ("NW");  
    }
    
  #ifndef CALIBRATION
    Serial.println(buff);
  #endif  
}

void showCalibration(){
  #ifdef DEBUG
    Serial.println("*--------LIST---------------------------*");
    Serial.print("x_min");
    Serial.print("\t (");
    Serial.print(x_min);
    Serial.print(") \t");
    Serial.print("x_max");
    Serial.print("\t (");
    Serial.println(x_max);
    Serial.print(")");
    Serial.print("y_min");
    Serial.print("\t (");
    Serial.print(y_min);
    Serial.print(") \t");
    Serial.print("y_max");
    Serial.print("\t (");
    Serial.println(y_max);
    Serial.print(")");
    Serial.print("z_max");
    Serial.print("\t (");
    Serial.print(z_max);
    Serial.println(")");
    Serial.println("*---------------------------------------*");
    Serial.println("Calibrated!");
  #endif
}
