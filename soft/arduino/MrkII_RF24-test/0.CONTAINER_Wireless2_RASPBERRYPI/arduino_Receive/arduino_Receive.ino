/*
 * ARDUINO RECEIVE DATA FROM CONTAINER WAIT FOR RASPBERRY PI TO READ DATA
 */
#define DEBUG
 
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

#define IRQ    2
#define CE     9
#define CSN   10

#define SLAVE_ADDR 0x08
#define DATA_SIZE 32

char receivedMessage[DATA_SIZE];
String dataPi;

// ce, csn pins
RF24 radio(CE,CSN);

void setup() {
  const uint64_t pipeWrite = 0xE8E8F0F0E1LL;
  const uint64_t pipeRead = 0xF0F0F0F0E1LL;
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

  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(requestEvent);
}

/*
 * Wait for a request from raspberry pi
 */

void requestEvent(){
  dataPi = "";
  for(int i = 0; i < DATA_SIZE; i++){
    if(receivedMessage[i]>= 32 and receivedMessage[i] <= 126){
      dataPi += (char)receivedMessage[i];
    }
    else if(dataPi == ""){
      dataPi = "empty";
    }
  }
  Wire.write(dataPi.c_str());
}

/*    
 *     Clear received message
 *     start listening
 */
 
void loop(){ 
  delay(10);
  receivedMessage[DATA_SIZE] = {0}; 
  radio.startListening();
  while(!radio.available());
  if(radio.available()){
    radio.read(receivedMessage,sizeof(receivedMessage));
#ifdef DEBUG
    Serial.println(receivedMessage);
#endif
  } 
     
  delay(10);
  String stringData(receivedMessage);
  if(stringData != ""){
    radio.stopListening();
    const char text[] = "ACK";
    radio.write(&text,sizeof(text));
#ifdef DEBUG
    Serial.print("SEND >>>> ");
    Serial.println(text); 
#endif     
  } 
  
}
