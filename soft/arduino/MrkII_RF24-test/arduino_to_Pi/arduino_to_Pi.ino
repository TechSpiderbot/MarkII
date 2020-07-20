#include <SPI.h>
#include <RF24.h>

#define IRQ    2
#define CE     9
#define CSN   10
bool received;

// ce, csn pins
RF24 radio(CE,CSN);

void setup() {
  const uint64_t pipeWrite = 0xF0F0F0F0E1LL;
  const uint64_t pipeRead = 0xE8E8F0F0E1LL;
  Serial.begin(115200);
  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(0x76);
  radio.openWritingPipe(pipeWrite);
  radio.openReadingPipe(1,pipeRead);
  radio.enableDynamicPayloads();
  radio.powerUp();

  received = false;
  Serial.println("Radio on");
}

void loop(){
  while(!received){
    //Serial.println("Send data");
    const char text[] = "Hello World";
    radio.write(&text,sizeof(text));  
  
    radio.startListening();
    delay(20);
    if(radio.available()){
      char receivedMessage[32] = {0};
      radio.read(receivedMessage,sizeof(receivedMessage));
      Serial.println(receivedMessage);
      Serial.println("Radio off");
      radio.stopListening();

      String stringMsg(receivedMessage);

      if(stringMsg == "ACK"){
        received = true;
      }
      else{
        received = false;
      }
    }
    radio.stopListening(); 
  }    
  delay(5000);   
  Serial.println("Radio on");
  received = false;
}
