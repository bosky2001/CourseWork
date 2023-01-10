#include <LSM6DS3.h>
#include <Wire.h>
#include "vive510.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>

//Setting up Sonar Pins
#define DistTrigger 4
#define DistEcho 5

//Setting I2C Pins
#define SDA1 15
#define SCL1 16

//Setting VIVE Pins 
#define RGBLED 18 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNALPIN1 5 // pin receiving signal from Vive circuit
#define UDPPORT 2510 // For GTA 2022C game 
#define STUDENTIP 100 // choose a teammembers assigned IP number
#define teamNumber 2
#define FREQ 1 // in Hz

//Setting up VIVE
Vive510 vive1(SIGNALPIN1);
esp_now_peer_info_t staffcomm = {
  .peer_addr = {0x84,0xF7,0x03,0xA9,0x04,0x78}, 
  .channel = 1,             // channel can be 1 to 14, channel 0 means current channel.
  .encrypt = false,
};

void pingstaff() {
  uint8_t teamNum = 5;
  esp_now_send(staffcomm.peer_addr, &teamNum, 1);     
}

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
float aX, aY, aZ, gX, gY, gZ;
const float accelerationThreshold = 0.5; // threshold of significant in G's
const int numSamples = 10;
int samplesRead = numSamples;

//Function for SONAR
float readDistanceSensor() {
   digitalWrite(DistTrigger, LOW);
   delayMicroseconds(2);
   digitalWrite(DistTrigger, HIGH);
   delayMicroseconds(10);
   digitalWrite(DistTrigger, LOW);
   float duration = pulseIn(DistEcho, HIGH);  // in microseconds
   float distance = duration * 0.0343 / 2.0;  // in centimeters
   return distance;
}

//Function for Accelerometer
void readAccelerometerSensor(){
  while (samplesRead == numSamples) {
    // read the acceleration data
    aX = myIMU.readFloatAccelX();
    aY = myIMU.readFloatAccelY();
    aZ = myIMU.readFloatAccelZ();

    // sum up the absolutes
    float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

    // check if it's above the threshold
    if (aSum >= accelerationThreshold) {
      // reset the sample read count
      samplesRead = 0;
      break;
    }
  }

  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  while (samplesRead < numSamples) {
    // check if both new acceleration and gyroscope data is
    // available
    // read the acceleration and gyroscope data

    samplesRead++;

    // print the data in CSV format
    aX = myIMU.readFloatAccelX();
    aY = myIMU.readFloatAccelY();
    aZ = myIMU.readFloatAccelZ();
    gX = myIMU.readFloatGyroX();
    gY = myIMU.readFloatGyroY();
    gZ = myIMU.readFloatGyroZ(); 
    

    if (samplesRead == numSamples) {
      // add an empty line if it's the last sample
      //Serial.println();
    }
  }
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  while (!Serial);
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    Serial.println("IMU Device error");
  } else {
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }

  WiFi.mode(WIFI_STA);  
  esp_now_init();      
  esp_now_add_peer(&staffcomm);  
  vive1.begin();
  Serial.println("Vive trackers started");
  
  pinMode(DistTrigger, OUTPUT);
  pinMode(DistEcho, INPUT);
  //Wire.setPins(SDA1, SCL1);  
}

void loop() {
  static long int ms = millis();
  static uint16_t x,y;
  

  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
      neopixelWrite(RGBLED,255,255,255);  // full white
    pingstaff();
  }
  
  if (vive1.status() == VIVE_RECEIVING) {
    x = vive1.xCoord();
    y = vive1.yCoord();
    neopixelWrite(RGBLED,0,x/200,y/200);  // blue to greenish
  }

  else {
    x=0;
    y=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
    }
  }
  
  
  readAccelerometerSensor();
  Serial.printf("Accelerometer: %4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f \n", aX, aY, aZ, gX, gY, gZ);
  Serial.printf("Sonar: %4.2f \n",readDistanceSensor());
  Serial.printf("VIVE: %4d,%4d", x,y);
  Serial.println("\n__________________________________\n");
  //}


  
  
  delay(20);
}
