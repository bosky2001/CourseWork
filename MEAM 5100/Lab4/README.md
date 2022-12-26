# Lab 4- Mobility

## 4.1 ESP32 AND Wi-Fi

### 4.1.2 Toggling LED using analog signals from the potentiometer

````cpp
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

ledcSetup(1,10,14);// increase 8 to 14 for lower frequency signals in this 10 so using 14
ledcAttachPin(5,1);
}

void loop() {
  // put your main code here, to run repeatedly:
 
    // changing the LED brightness with PWM
    int adc=analogRead(4);
    int dutyCycle=(4*adc);
    
    ledcWrite(1, dutyCycle);

}
````

### 4.1.3 Website to display and control the duty cycle and frequency of LED
Webpage code
````cpp
const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<body>
<div class="topnav">
        <h1>Lab 4.1.3</h1>
    </div>
    <div class="content">
        <div class="card-grid">
            <div class="card">
                <p class="card-title">LED Brightness</p>
                <p class="switch">
                    <input type="range"  id="slider1" min="0" max="100" step="1" value ="80" class="slider1">
                </p>
                <p class="state">Brightness: <span id="sliderValue1"></span> &percnt;</p>
            </div>
            <div class="card">
                <p class="card-title"> Frequency Control</p>
                <p class="switch">
                    <input type="range"  id="slider2" min="3" max="30" step="1" value ="20" class="slider2">
                </p>
                <p class="state">Frequency: <span id="sliderValue2"></span> Hz</p>
            </div>
            
        </div>
    </div>
</body>
<script>
slider1.onchange = function dutyFunction () {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("sliderValue1").innerHTML = this.responseText;
      }
    };
    var str = "slider1?val=";
    var res = str.concat(this.value);
    xhttp.open("GET", res, true);
    xhttp.send();
  }
    
  slider2.onchange = function freqFunction() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("sliderValue2").innerHTML = this.responseText;
      }
    };
    var str = "slider2?val=";
    var res = str.concat(this.value);
    xhttp.open("GET", res, true);
    xhttp.send();
  }
</script>
</html>
)===";
````

Including the webpage as a header in the main file to be run

[**Youtube Demo**](https://youtube.com/shorts/8vuF2B7Yjb0)
````cpp
#include <WiFi.h>
#include <WiFiUDP.h>
#include "myweb.h"
#include "html510.h"

HTML510Server h(80);
// WiFi name
IPAddress myIP(192,168,1,170);      // initialize the local IP address
const char* ssid="WiFi ID";       // this is the network name
const char* pass = "password";         // this is the password

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
  int duty = h.getVal();
  
  uint32_t cycle = 2.55*duty;
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

  ledcSetup(0, 20, 14);//start it at 20 Hz
  ledcAttachPin(4, 0);
  
  pinMode(5,OUTPUT);
  
  //setup dutycycle and frequency slider handles
  h.begin();
  h.attachHandler("/ ",handleRoot);
  h.attachHandler("/slider1?val=",handleDuty);
  h.attachHandler("/slider2?val=",handleFreq);
  
  
}

void loop(){
  pinMode(5,OUTPUT);
  digitalWrite(5,HIGH);
  h.serve();
  delay(10);
  digitalWrite(5,LOW);
} 

````

### 4.1.4 Variable Direction and Speed of DC Motor
Webpage to control the Motor direction and speed
````c
const char body[] PROGMEM=R"===(
<!DOCTYPE html>
<html>
  <body>
    <a href="/L">Backwards </a> <br>
    
    <a href="/H">Forwards  </a> <br>
    
    
    <br>
    <input type="range" min="1" max="100" value="1"  id="slider">
    <span id="sliderlabel">  </span> <br>
  </body>
  <script>
    function hit() {
      var xhttp = new XMLHttpRequest();
      xhttp.open("GET", "hit", true);
      xhttp.send();
    }
    setInterval(updateLabel, 1000);
    updateLabel();
    function updateLabel() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          document.getElementById("buttonlabel").innerHTML = this.responseText;
        }
      };
      xhttp.open("GET", "LEDstate", true);
      xhttp.send();
    }
    slider.onchange = function() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          document.getElementById("sliderlabel").innerHTML = this.responseText;
        }
      };
      var str = "slider?val=";
      var res = str.concat(this.value);
      xhttp.open("GET", res, true);
      xhttp.send();
    }
  </script>
</html>
)===";
````
[Youtube Demo](https://youtu.be/QHf7AhsBG00): 


Including the corresponding header in the main file
````cpp
#include "html510.h"
#include "webpage2.h"
HTML510Server h(80);

#define PWM3 4
#define DIR3 10
#define DIR4 18
const char* ssid="WiFi ID";
const char* pass ="password";

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
````

