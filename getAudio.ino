#include <WiFiNINA.h>
#include <PDM.h>

bool LED_SWITCH = false;

// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits

static const int BUFFER_SIZE = 32000;

short sampleBuffer[BUFFER_SIZE];

bool readSamples = false;
bool begining = false;

int i = 0;

// Number of audio samples read
volatile int samplesRead = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LEDB, OUTPUT);
  while (!Serial);
  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and -10 on the Portenta Vision Shields
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shields
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
}

void loop() {
  // Wait for samples read to be done
  int record = Serial.read();
  if(record == 97){ //if its lowercase a
    Serial.println("recording!");
    readSamples = true;
    begining = true;
  }
  
  // Wait for samples read to be done
  if(!readSamples && begining){
    while (i < BUFFER_SIZE) {
        Serial.println(sampleBuffer[i]);
        i = i+1;
      }
      
    begining = false;
    samplesRead = 0;
    digitalWrite(LEDB,LOW);
    i = 0;
  }
}

/**
   Callback function to process the data from the PDM microphone.
   NOTE: This callback is executed as part of an ISR.
   Therefore using `Serial` to print messages inside this function isn't supported.
 **/
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  if (readSamples) {
    int toRead = bytesAvailable/2;
    toRead = min (toRead, BUFFER_SIZE - samplesRead);
    PDM.read(sampleBuffer + samplesRead, toRead*2);
    samplesRead += toRead;
    readSamples = (samplesRead < BUFFER_SIZE);
  }
}
