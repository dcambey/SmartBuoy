#include <WiFiNINA.h>
#include <PDM.h>
#include <arduinoFFT.h>
#include <RadioLib.h>

arduinoFFT FFT;

// default number of output channels
static const char channels = 1;

// default PCM output frequency
double frequency = 16000;

const int numSamples = 8192;

// Buffer to read samples into
short sampleBuffer[numSamples];

double vImag[numSamples];
double vReal[numSamples];

double min_freq[2] = {600,1000}; //Hz, min for boat noise on spectogram
double max_freq[2] = {1400, 3000}; //Hz, max for boat noise on spectogram
double boatRatio_cutoff[2] = {0.2, 0.35};

bool readSamples = false;
bool begining = false;

// Number of audio samples read
volatile int samplesRead = 0;

//values for LoRa comunication
// SX1276 requires the following connections:
int pin_cs = 10;
int pin_dio0 = 2;
int pin_nrst = 9;
int pin_dio1 = 4;
SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);

void setup() {
//  Serial.begin(9600);
  pinMode(LEDB, OUTPUT);
//  while (!Serial);
  // Configure the data receive callback
  PDM.onReceive(onPDMdata);
  // Optionally set the gain

  // Initialize PDM, 1 channel
  if (!PDM.begin(channels, frequency)) {
      Serial.println("Failed to start PDM!");
      while (true);
    }

  //initialize LoRa
  Serial.print(F("Radio Initializing ... "));
  int state = radio.begin(915.0); //-23dBm
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("init success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  if (radio.setOutputPower(20) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    Serial.println(F("Selected output power is invalid for this module!"));
    while (true);
  }

  int pin_rx_enable = 6;
  int pin_tx_enable = 5;
  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);

  //if everything went well
  Serial.println("Setup Complete!");
}

void loop() {
  //get the number from the receiver
  if(!readSamples && !begining){ //mode where getting starting signal from receiver
  
    String starting_sig;
    int state = radio.receive(starting_sig);
    if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully transmitted
        digitalWrite(LEDB,LOW);
        Serial.println(F("Starting signal received!"));
        Serial.print(F("Signal Strength:\t\t\t"));
        Serial.println(radio.getRSSI());
        Serial.println(starting_sig);
    }
     else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      // timeout occurred while waiting for a packet
//      Serial.println(F("timeout!"));
  
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
//      Serial.println(F("CRC error!"));
  
    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
      
    if(starting_sig == "1"){ //if starting val sent from receiver
      Serial.println("recording!");
      readSamples = true;
      begining = true;
      //send it to read samples
    }
  }

//   Wait for samples read to be done
  if(!readSamples && begining){ //if it is done recording sampels
   //construct vReal and vImag
    for(int i=0;i < numSamples;i++){
      vReal[i] = sampleBuffer[i];
      vImag[i] = 0;
    }

    //compute FFT
    FFT = arduinoFFT(vReal, vImag, numSamples, frequency); /* Create FFT object */

    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */

    FFT.Compute(FFT_FORWARD); /* Compute FFT */

    FFT.ComplexToMagnitude(); /* Compute magnitudes */

    //compute boat ratio
    double withinBand = 0;
    double outsideBand = 0;
    double freq = 0;
    double boatRatio[2];
    for(int j = 0; j < 2; j++){
      for (int i = 0; i < numSamples/2; i++) {
        freq = i*frequency/numSamples;
        if(freq < max_freq[j] && freq > min_freq[j]){
         withinBand = withinBand + vReal[i];
        }
        else{
         outsideBand = outsideBand + vReal[i];
        }
      }
      boatRatio[j] = withinBand/(withinBand+outsideBand);
      
      withinBand = 0;
      outsideBand = 0;
    }
   Serial.println("low end boat ratio:");
   Serial.println(boatRatio[0],6);
   Serial.println("high end boat ratio:");
   Serial.println(boatRatio[1],6);
   bool boat = 0;
   if(boatRatio[0] > boatRatio_cutoff[0] && boatRatio[1] > boatRatio_cutoff[1]){
    boat = 1;
    Serial.println("Boat detected!");
   }
   else{
    Serial.println("No boat detected");
   }
   
   
   //send the data
   Serial.print(F("[SX1276] Transmitting packet ... "));
   int state = 0;
   Serial.println(boat);
   digitalWrite(LEDB,HIGH);
   delay(1000);

   String lowEnd_toSend = "";
   lowEnd_toSend = String(boatRatio[0],5);
   String highEnd_toSend = "";
   highEnd_toSend = String(boatRatio[1],5);
   state = radio.transmit(lowEnd_toSend);
   delay(500)
   state = radio.transmit(highEnd_toSend);
   delay(500)
   
   if(boat){
    state = radio.transmit("1");
    digitalWrite(LEDB,HIGH);
   }
   else{
    state = radio.transmit("0");
    digitalWrite(LEDB,LOW);
   }

   if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));
   }
    
  //reset constants
  begining = 0;
  samplesRead = 0;
  boat = 0;
  
  }
}

//callback function that gets executed on the ISR
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  if (readSamples) {
    int toRead = bytesAvailable/2;
    toRead = min (toRead, numSamples - samplesRead);
    PDM.read(sampleBuffer + samplesRead, toRead*2);
    samplesRead += toRead;
    readSamples = (samplesRead < numSamples);
  }
}
