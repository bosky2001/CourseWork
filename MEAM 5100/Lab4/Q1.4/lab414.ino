
#include "html510.h"
#include "webpage2.h"
HTML510Server h(80);

#define PWM3 4
#define DIR3 10
#define DIR4 18
const char* ssid="APT207";
const char* pass ="walnut207";

volatile int prevduty=0;
void handleRoot() {
  h.sendhtml(body);
}
void handleSlider(){
  int dutyCycle;  // Current Duty Cycle
  int sliderValue = h.getVal();   // Get Slider Value

  // Website Report Status (DEBUG)
  String s = "Slider ";
  s = s+String(sliderValue)+"%";  // Add Slider Value to Website Status String
  s = s+", DutyCycle ";
  dutyCycle = 165+0.91*sliderValue;       // Equation for getting the motor to run from low duty values as voltage into motor is not same as whatever is coming from the channel
  s = s+String(dutyCycle);  // Add Duty Cycle to Website Status String
  h.sendplain(s); // Send updated values back to website
  ledcWrite(1,dutyCycle);      // updating the channel with current slider value reading for duty cycle
}

void handleH(){
  Serial.println("Dir is forward"); // LED ON

  digitalWrite(DIR3,HIGH); // setting 1A to high and 2A to low for first config mode( forward/clockwise rotation)
  digitalWrite(DIR4,LOW);
  
  h.sendhtml(body); // acknowledge   
}

void handleL(){
  
  Serial.println("Dir is reverse");// LED OFF
  digitalWrite(DIR3, LOW);            // setting 2A to high and 1A to low for second config mode( backward/counter-clockwise rotation)
  digitalWrite( DIR4,HIGH);
  
  h.sendhtml(body); // acknowledge   
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(DIR3,OUTPUT);             // setting the pins mode to output
  pinMode(DIR4,OUTPUT);
  
  
  ledcSetup(1,20000,8);  // setting up timer or pwm channel
  ledcAttachPin(4,1);    // attaching the gpio to the timer/ pwm channel

  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid,pass);

  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.print("Use this URL to connect: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/"); 

  h.begin();  // Begin hosting WebSite
  h.attachHandler("/ ",handleRoot);
  // direction_Button_Handlers
  h.attachHandler("/H",handleH);
  h.attachHandler("/L",handleL);
  
  // PWM_Slider_Handler
  h.attachHandler("/slider?val=",handleSlider);
}

void loop() {
  // put your main code here, to run repeatedly:
  h.serve();
  delay(10);
}
