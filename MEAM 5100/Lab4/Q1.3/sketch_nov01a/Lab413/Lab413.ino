//WIFI setup
#include <WiFi.h>
#include <WiFiUDP.h>
#include "myweb.h"
#include "html510.h"

HTML510Server h(80);
// WiFi name
IPAddress myIP(192,168,1,170);      // initialize the local IP address
const char* ssid = "APT207";      // this is the network name
const char* pass = "walnut207";        // this is the password

#define LEDC_CHANNEL       0
#define LEDC_RESOLUTION_BITS  14
#define LEDC_RESOLUTION  ((1<<LEDC_RESOLUTION_BITS)-1) 
#define LED_PIN           4



/*****************/
/* web handler   */
void handleRoot() {
  h.sendhtml(body);
}

//get duty from slider and write to LED_PIN
void handleDuty(){
  float duty = h.getVal();
  
  uint32_t cycle = 163.84*duty;                      // using 14 bit resolution so duty cycle max is 16384 instead, multiplying with 163.84 as a sacling factor to 0-100 range inputs
  String s =  String(duty);
  ledcWrite(LEDC_CHANNEL,cycle); 
  h.sendplain(s);
}

//get frequency from slider and write to LED_PIN
void handleFreq(){
  double freq = double(h.getVal());
  String s = String(freq);
  ledcSetup(LEDC_CHANNEL, freq, LEDC_RESOLUTION_BITS); //when frequency is changed, reset LEDC setup
  //ledcAnalogWrite(LEDC_CHANNEL,75); //make sure to re-output duty
  h.sendplain(s);
}

// IP Addresses



void setup(){ 
  Serial.begin(115200);
  
  WiFi.begin(ssid,pass);
  while(WiFi.status()!=WL_CONNECTED){
  delay(500);
  Serial.print(".");
  }
  Serial.print("Use this URL to connect: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/"); 

  ledcSetup(LEDC_CHANNEL, 20, LEDC_RESOLUTION_BITS);//start it at 20 Hz
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);
  
  
  
  //setup dutycycle and frequency slider handles
  h.begin();
  h.attachHandler("/ ",handleRoot);
  h.attachHandler("/slider1?val=",handleDuty);
  h.attachHandler("/slider2?val=",handleFreq);
  
  
}

void loop(){
  
  h.serve();
  delay(10);
  
} 
