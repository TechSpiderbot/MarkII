/*
 * ARDUINO SEND data from CONTAINER TO RASPBERR PI
 */
#define DEBUG
 
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

#define IRQ    2
#define CE     9
#define CSN   10

#define SLAVE_ADDR 0x09
#define DATA_SIZE 32

// ce, csn pins
RF24 radio(CE,CSN);

static char nucleoData[DATA_SIZE] = "";
bool dataReceived;
int resetCounter = 0;

// reset arduino when does't get an ack
void(* resetFunc) (void) = 0;

void setup() {
  const uint64_t pipeWrite = 0xF0F0F0F0E1LL;
  const uint64_t pipeRead = 0xE8E8F0F0E1LL;
#ifdef DEBUG
  Serial.begin(115200);
#endif
  
  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(0x76);
  radio.openWritingPipe(pipeWrite);
  radio.openReadingPipe(1,pipeRead);
  radio.enableDynamicPayloads();
  radio.startListening();

  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  dataReceived = false;
}

void requestEvent(){}
/*
 * receive data from Nucleo container
 */
void receiveEvent(){
  while(0< Wire.available()){
    for(int i = 0; i < DATA_SIZE; i++){
      nucleoData[i] = Wire.read();
    }
  }
  dataReceived = true;
  //Serial.print("Nucleo RECEIVED >>>> ");
  //Serial.println(nucleoData);
}

void loop(){
  delay(5);

  /*
   * Wait till data is received
   */
  if(dataReceived){
    dataReceived = false;
    
    radio.stopListening();
    //const char text[] = "Hello World";
    radio.write(&nucleoData,sizeof(nucleoData)); 
#ifdef DEBUG
    Serial.print("SEND >>>> ");
    Serial.println(nucleoData); 
#endif
    delay(5);

    radio.startListening();
    // wait till receive an replay
    while(!radio.available()){
        delay(10);
        resetCounter++;
#ifdef DEBUG        
        Serial.println(resetCounter);
#endif         
        if(resetCounter >= 4){ // 5 sec time-out
          Serial.println("RESET"); 
          delay(10);
          resetFunc();  
        }
    } 

    if(radio.available()){
      // RESET resetCounter
      resetCounter = 0;               
      char receivedMessage[DATA_SIZE] = {0};
      radio.read(receivedMessage,sizeof(receivedMessage));
      
      String stringData(receivedMessage);
      if(stringData == "ACK"){
#ifdef DEBUG
        Serial.println(receivedMessage);
#endif        
      } 
    }
  }
}
