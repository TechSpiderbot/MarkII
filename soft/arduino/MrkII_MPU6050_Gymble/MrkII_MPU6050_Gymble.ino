/*
* MASTER Arduino
* Serial communication between container 
* 
* I2C comunication to slaves
* 
*/
#include <RGBLed.h>
/* https://github.com/wilmouths/RGBLed
*/
#include "stdio.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// ================================================================
// ===               #DEFINE                                    ===
// ================================================================
/* Control keys for reading data
 * ack = enable to read data
 * dis = disable to read data
 *  
 *  the adruino nano will continiue run even when disabled, only the data request is disabled.
 */
#define enableGyroData "ack\n"    // receive confirm 10
#define disableGyroData "dis\n"   // receive confirm 0

#define MAX_OUT_CHARS 16

/* request Gyro data
 *  Yaw   data send i
 *  Pitch data send j
 *  Roll  data send k
 */
#define accesYaw "i\n"
#define accesPitch "j\n"
#define accesRoll "k\n"

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// ================================================================
// == Master settings
// ================================================================
/* request head distance data:
 *  demo mode acces with h
 *  
 * request leg sensor data: 
 *  all data update       l
 *  pick serial separated p
 */

#define accesHeadSensor "h\n"
#define accesLegSensor  "l\n"
#define accesLegSeperated "p\n"
/*
 * Define Slave I2C Address
 */
#define HEAD_ADDR 0x06
#define MASTER_ADDR 0x07
#define COMPASS_ADDR 0x08
#define DISTANCE_VALVE 3

// Define counter to count bytes in response
int bcount;
// Define array for return data
int SensorDistance[DISTANCE_VALVE];
int SensorLEGS[8];
int iSensorLEGS;

/*
 * RGB led settings
 */
bool I2C_error = false;
 
#define DATA_SIZE 2
int nucleoData[DATA_SIZE];
int LEG1_DATA[DATA_SIZE];
int LEG2_DATA[DATA_SIZE];
// RGB LED CONFIG
//            RED, GREEN, BLUE
int LEG1_RGB[3] = {11,10,9};
int LEG2_RGB[3] = {6,5,3};

RGBLed led1(LEG1_RGB[0], LEG1_RGB[1], LEG1_RGB[2],COMMON_CATHODE);
RGBLed led2(LEG2_RGB[0], LEG2_RGB[1], LEG2_RGB[2],COMMON_CATHODE);
 
// ================================================================
// == End master settings
// ================================================================


// ================================================================
// == MPU settings
// ================================================================
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

float correct;
int j = 0;

int num = 0;
char  buffer[MAX_OUT_CHARS + 1];

// serial event variable
String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool getData = false;

int outp0Value = 0;
int outp1Value = 0;
int outp2Value = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float outpValue[3];
// ================================================================
// packet structure for InvenSense teapot demo
// uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
// ================================================================


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
// reset arduino when does't get an ack
void(* resetFunc) (void) = 0;

void setup() {
  
  // Setup Serial and basic I2C
  // ================================================================
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  
  // Setup Gyroscoop
  // ================================================================
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }
  Wire.begin(MASTER_ADDR);
  Wire.onReceive(receiveEvent);
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void SetColour();
void CHECK_FLAGS();

void readGyroData();
int readI2C(int);
void readHeadSensorDatarrwd();
void readLegSensorData();

void loop() {
   
  readGyroData();    //READ GYRO DATA 
  
  // send only data on request
  /*
   * IF adruino get an ack\n then 
   * Be active to comunicate by serial
   */
  if (stringComplete){
    //Serial.print(inputString);
    if(inputString == enableGyroData){ //ENABLE: send "ack\n" 
      num = 10;
      sprintf(buffer,"%d",num);
      Serial.println(buffer);
      getData = true;   
    }
    else if(inputString == disableGyroData){ //DISABLE: send "dis\n"
      num = 0;
      sprintf(buffer,"%d",num);
      Serial.println(buffer);
      getData = false; 
    }

    if(getData){
      // == Gyro data ==
      if(inputString == accesYaw){ // I: Send YAW
        //Serial.print("Yaw: ");   
        sprintf(buffer,"%d",outp0Value);
        Serial.println(buffer);
      }
      else if(inputString == accesPitch){ // J: Send Pitch
        //Serial.print("Pitch: "); 
        sprintf(buffer,"%d",outp1Value);
        Serial.println(buffer);
      }
      else if(inputString == accesRoll){ // K: Send Roll
        //Serial.print("Roll: ");  
        sprintf(buffer,"%d",outp2Value);
        Serial.println(buffer);
      } 
      // == End Gyro data ==

      // == Get Slave data on request only ==
      else if(inputString == accesHeadSensor){ //H: Request Head Sensor data
        readHeadSensorData();
        // temporary it's send with tabs in between
        for (int i = 0; i < DISTANCE_VALVE; i++) {
          sprintf(buffer,"%d",SensorDistance[i]);
          Serial.print(buffer);
          Serial.print("\t");
        }
        Serial.println();
      }
      else if(inputString == accesLegSensor){ //L: Request Leg Sensor data
        readLegSensorData();
        // temporary it's send with tabs in between
        for (int i = 0; i < 9; i++) {
          sprintf(buffer,"%d",SensorLEGS[i]);
          Serial.print(buffer);
          Serial.print("\t");
        }
        
        Serial.println();
        iSensorLEGS = 1;
      }
      else if(inputString == accesLegSeperated){ //P: Request Leg Sensor data  
          sprintf(buffer,"%d",SensorLEGS[iSensorLEGS]);
          Serial.println(buffer);
          iSensorLEGS++;
          if(iSensorLEGS > 8){
            iSensorLEGS = 1;
          }
      }
    } 

    // clear inputString and stringComplete
    inputString = "";
    stringComplete = false;
  }
}

void receiveEvent(){
  // clear nucleoData[]
  for(int i = 0; i < DATA_SIZE; i++){
      nucleoData[i] = 0;
  }

  // read incomming data
  while(0< Wire.available()){
    for(int i = 0; i < DATA_SIZE; i++){
      nucleoData[i] = Wire.read();

      if(nucleoData[0] == 1){
        LEG1_DATA[i] = nucleoData[i];
      }
      else if(nucleoData[0] == 2){
        LEG2_DATA[i] = nucleoData[i];
      }
    }
  }
  SetColour();
}

void SetColour(){
  /* 
  Serial.print("LEG3_DATA: \t");
  for(int i =0; i < DATA_SIZE; i++){
    Serial.print(LEG1_DATA[i]);
    Serial.print("\t");
  }
  Serial.println("");  
    
  Serial.print("LEG4_DATA: \t");
  for(int i =0; i < DATA_SIZE; i++){
    Serial.print(LEG2_DATA[i]);
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
  if( LEG1_DATA[0] == 1 && LEG1_DATA[1] == 1){ // blink
    led1.fadeOut(RGBLed::RED,5,100);
  }
  else {
    // LIGHT OFF
      led1.off();
  }
    
  if(LEG2_DATA[0] == 2 && LEG2_DATA[1] == 1){ // blink
    led2.fadeOut(RGBLed::RED,5,100);
  }
  else{
    led2.off();
  }
}

// ================================================================
// ===                    SERIAL EVENT                          ===
// ================================================================
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // temp print
    //Serial.println(inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// ================================================================
// ===                    I2C EVENT HEAD Sensor                 ===
// ================================================================
void readLegSensorData(){
  /*
  int i = 0;
  while (readI2C(COMPASS_ADDR) < 255) {
  // Until first byte has been received print a waiting message
    delay(100);
    i++;
    if(i >= 20){
      break;
      Serial.println("Error");
    }
  }
  for (bcount = 0; bcount < 9; bcount++) {
    SensorLEGS[bcount] = readI2C(COMPASS_ADDR);
  }  
  */
  Wire.requestFrom(COMPASS_ADDR, 9); 
  int i = 0;
  while(Wire.available()){
    SensorLEGS[i] = Wire.read();
    i++;
  }
}

void readHeadSensorData(){
  int i = 0;
  while (readI2C(HEAD_ADDR) < 255) {
  // Until first byte has been received print a waiting message
    delay(100);
    i++;
    if(i >= 20){
      break;
      Serial.println("Error");
    }
  }
  for (bcount = 0; bcount < DISTANCE_VALVE; bcount++) {
    SensorDistance[bcount] = readI2C(HEAD_ADDR);
  } 
}

int readI2C(int address) {
  // Define a variable to hold byte of data
  int bval;
  long entry = millis();
  
  // Read one byte at a time
  Wire.requestFrom(address, 1); 
  while(Wire.available()== 0 && (millis() - entry) < 100){}
  if(millis() - entry < 200){
    bval = Wire.read();
  }
  return bval;
}

// ================================================================
// ===                    READ GYRO DATA                        ===
// ================================================================

void readGyroData(){
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    //Serial.println(F("FIFO overflow!"));
  
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
  
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  
    // Get Yaw, Pitch and Roll values
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
    // Yaw, Pitch, Roll values - Radians to degrees
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
      
    // Skip 300 readings (self-calibration process)
    if (j <= 300) {
      correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
      j++;
    }
    // After 300 readings
    else {
      ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees
      
      // Map the values of the MPU6050 sensor from -90 to 90 to values suatable for the servo control from 0 to 180
      outp0Value = map(ypr[0], -90, 90, 0, 180);
      outp1Value = map(ypr[1], -90, 90, 0, 180);
      outp2Value = map(ypr[2], -90, 90, 0, 180); 
    }
  } 
}
