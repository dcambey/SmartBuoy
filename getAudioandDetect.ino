#include <WiFiNINA.h>
#include <PDM.h>
#include <arduinoFFT.h>

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

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup() {
  Serial.begin(9600);
  pinMode(LEDB, OUTPUT);
  while (!Serial);
  // Configure the data receive callback
  PDM.onReceive(onPDMdata);
  Serial.println("starting!");
  // Optionally set the gain

  // Initialize PDM, 1 channel
  if (!PDM.begin(channels, frequency)) {
      Serial.println("Failed to start PDM!");
      while (1);
    }
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
    for(int i = 0; i < numSamples; i++){
      vReal[i] = sampleBuffer[i];
      vImag[i] = 0;
    }
    
    FFT = arduinoFFT(vReal, vImag, numSamples, frequency); /* Create FFT object */
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    FFT.Compute(FFT_FORWARD); /* Compute FFT */
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
