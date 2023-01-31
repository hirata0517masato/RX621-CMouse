#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

String MACadd = "E0:5A:1B:5F:1C:42";
uint8_t address[6]  = {0xE0, 0x5A, 0x1B, 0x5F, 0x1C, 0x42};

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test", true); 

  pinMode( 2, OUTPUT);
  //pinMode(14, INPUT_PULLUP);
  pinMode(14, INPUT);
  
  while(!SerialBT.connect(address)){
    Serial.println("try connect");
  } 

  digitalWrite(2, HIGH);
}

void loop() {
 
  //Serial.println(analogRead(14));

   while(1){
    if(SerialBT.available()){
     // Serial.println((char)SerialBT.read());
      if ((char)SerialBT.read() == 's') {
        break;
      }
    }
  }

  while(1){
    if (analogRead(14) > 36) {
    //  Serial.println('H');
      SerialBT.write('H');
    }else{
    //  Serial.println('L');
      SerialBT.write('L');
    }

    if(SerialBT.available()){
      if ((char)SerialBT.read() == 'g') {
        break;
      }
    }
    //delay(1);
  }
  
}
