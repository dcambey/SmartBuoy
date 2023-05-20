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
double boatRatio_cutoff[2] = {0.15, 0.4};


bool readSamples = false;
bool begining = false;

// Number of audio samples read
volatile int samplesRead = 0;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

//values for LoRa comunication
// SX1276 requires the following connections:
int pin_cs = 10;
int pin_dio0 = 2;
int pin_nrst = 9;
int pin_dio1 = 4;
SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);

void setup() {
  Serial.begin(9600);
  pinMode(LEDB, OUTPUT);
  while (!Serial);
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
 
  int record = Serial.read();
  if(record == 97){ //if its lowercase a
    Serial.println("recording!");
    readSamples = true;
    begining = true;
  }
  
  // Wait for samples read to be done
  if(!readSamples && begining){
    Serial.println("done recording!");
   //construct vReal and vImag
    for(int i = 0;i < numSamples;i++){
      vReal[i] = sampleBuffer[i];
      vImag[i] = 0;
    }
    
    FFT = arduinoFFT(vReal, vImag, numSamples, frequency); /* Create FFT object */
    //print the data and results
    Serial.println("Data:");
    PrintVector(vReal, numSamples, SCL_TIME);
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  //  Serial.println("Weighed data:");
  //  PrintVector(vReal, numSamples, SCL_TIME);
    FFT.Compute(FFT_FORWARD); /* Compute FFT */
  //  Serial.println("Computed Real values:");
  //   delay(1000);
  //  PrintVector(vReal, numSamples, SCL_INDEX);
  //  Serial.println("Computed Imaginary values:");
  //  delay(1000);
  //  PrintVector(vImag, numSamples, SCL_INDEX);
    FFT.ComplexToMagnitude(); /* Compute magnitudes */
    Serial.println("Computed Spectogram:");
    delay(600);
    
    double withinBand = 0;
    double outsideBand = 0;
    double freq = 0;
    double boatRatio[2];
    PrintVector(vReal, (numSamples >> 1), SCL_FREQUENCY);

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

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / frequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * frequency) / numSamples);
  break;
    }
    Serial.print(abscissa, 2);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
