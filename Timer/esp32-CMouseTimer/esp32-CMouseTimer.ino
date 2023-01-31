#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

int pin_7seg[12] = {18,19,21,13,12,27,5,17,16,4,0,15};
char numberPin[7] = {10,6,3,1,0,9,4};
char botPin = 2;
char corPin[4] = {5,7,8,11};
char numberList[10][7] = {
    {0,0,0,0,0,0,1}, //0
    {1,0,0,1,1,1,1}, //1
    {0,0,1,0,0,1,0}, //2
    {0,0,0,0,1,1,0}, //3
    {1,0,0,1,1,0,0}, //4
    {0,1,0,0,1,0,0}, //5
    {0,1,0,0,0,0,0}, //6
    {0,0,0,1,1,0,1}, //7
    {0,0,0,0,0,0,0}, //8
    {0,0,0,0,1,0,0}  //9
};

void setup_7seg(){
  for(int i = 0; i < 12;i++){
    pinMode( pin_7seg[i], OUTPUT);
    digitalWrite(pin_7seg[i], LOW);
  }
}

void set_7seg(int n,int dot){
  int m;
  for(int j = 0;j<12;j++){
    digitalWrite(pin_7seg[j],0); //all 0
  }
    
  for(int i = 0; i < 4;i++){
   
    if(dot != i)digitalWrite(pin_7seg[botPin],1);
    else digitalWrite(pin_7seg[botPin],0);

    m = n%10;
    for(int j = 0;j < 7;j++){
      digitalWrite(pin_7seg[numberPin[j]], numberList[m][j]);
    }
    n /= 10;
    
    digitalWrite(pin_7seg[corPin[i]],1);
    delay(5);
    digitalWrite(pin_7seg[corPin[i]],0);
  }
}

void set_7seg_start(){
  for(int j = 0;j<12;j++){
    digitalWrite(pin_7seg[j],0); //all 0
  }

  for(int j = 0;j < 7;j++){
    digitalWrite(pin_7seg[numberPin[j]], 1);
  }
  
  for(int i = 0; i < 4;i++){
    digitalWrite(pin_7seg[corPin[i]],1);
  }
}
int get_SW(){
  if(digitalRead(23) == LOW){
    return 1;
  }else{
    return 0;
  }
}

void set_time(unsigned long diff){

  int cnt = 0;
  unsigned long tmp = diff;
  int dot = 3;
  
  while(tmp > 0){
    cnt++;
    tmp /= 10;
  }

  for(int i =0 ;i< cnt-4;i++){
    diff /= 10;
    dot--;
  }
  
  set_7seg(diff,dot);
}
/*
void setup(void) {
  Serial.begin(115200);
  Serial.println("-----------------");
  uint8_t macBT[6];
  esp_read_mac(macBT, ESP_MAC_BT);
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\r\n", macBT[0], macBT[1], macBT[2], macBT[3], macBT[4], macBT[5]);
}*/

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode( 2, OUTPUT);
  pinMode(14, INPUT);
  setup_7seg();
  pinMode(23, INPUT);
  
  digitalWrite(2, HIGH);

}

void loop() {

/*  while(1){
    if(SerialBT.available()){
      break;
    }
  }
  set_7seg_start();
  delay(1000);
*/  
  //buff clear
  while(1){
    if(SerialBT.available()){
     Serial.println((char)SerialBT.read());
    }else{
      Serial.println("OK");
      break;
    }
  }
  
  set_7seg(0,0);
  
  while(get_SW() != 1);
  while(get_SW() == 1);

  set_7seg_start();

  SerialBT.write('s');
  while(analogRead(14) > 36);
  //start
  previousMillis = millis();
  
  set_7seg(0,0);
  
  
  while(1){
    if(get_SW() == 1){//reset
      currentMillis = millis();
      SerialBT.write('g');
      break;
    }
    if(SerialBT.available()){
     // Serial.println((char)SerialBT.read());
      if ((char)SerialBT.read() == 'L') {
        //stop
        currentMillis = millis();
        
        SerialBT.write('g');
        break;
      }
    }
  }
  
  while(get_SW() != 1){
    set_time(currentMillis - previousMillis);
  }
  while(get_SW() == 1);
  set_7seg(0,0);
  delay(100);

  //buff clear
  while(1){
    if(SerialBT.available()){
     Serial.println((char)SerialBT.read());
    }else{
      Serial.println("OK");
      break;
    }
  }
}
