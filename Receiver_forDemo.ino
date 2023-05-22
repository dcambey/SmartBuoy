// include the library
#include <RadioLib.h>

bool starting = true;

int i = 0;

// SX1276 requires the following connections:
int pin_cs = 10;
int pin_dio0 = 2;
int pin_nrst = 9;
int pin_dio1 = 6;
int  LED_PIN = 3;
SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN,OUTPUT);

  while(!Serial);
  
  // initialize SX1278 with default settings
  Serial.print(F("Initializing Electronics ... "));
  int state = radio.begin(915.0);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

   if (radio.setOutputPower(20) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    Serial.println(F("Selected output power is invalid for this module!"));
    while (true);
  }

  // some modules have an external RF switch
  // controlled via two pins (RX enable, TX enable)
  int pin_rx_enable = 7;
  int pin_tx_enable = 5;
  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);

  Serial.println("Waiting for input");
  Serial.println("  ");
}

void loop() {
  
  if(starting){
    int state = 10;
      if(i == 0){
        Serial.println("Sending signal to record!");
      }
      String str = "1";
      state = radio.transmit(str);
      
      if (state == RADIOLIB_ERR_NONE) {
      }
      else{
      Serial.println(F("signal not transmitted"));
      }
      
      i = i+1;
      if(i > 10){
        starting = false;
      }
    }
    
  else{ //if receiving a signal

    String string;
    int state = radio.receive(string);
    
    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.print(F("Signal received! Strength:\t\t\t"));
      Serial.println(radio.getRSSI());
      
      Serial.println(string);
      
      digitalWrite(LED_PIN,HIGH);
      if(string == "1"){
      Serial.println("boat found!");
      digitalWrite(LED_PIN,HIGH);
      }
      else{
        Serial.println("no boat found");
        digitalWrite(LED_PIN,LOW);
      }
      
    }
    
  }
  
  }
