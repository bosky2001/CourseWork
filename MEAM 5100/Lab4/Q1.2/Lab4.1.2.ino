void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

ledcSetup(1,10,14);// increase 8 to 14/20 maybe
ledcAttachPin(5,1);
}

void loop() {
  // put your main code here, to run repeatedly:
 
    // changing the LED brightness with PWM
    int adc=analogRead(4);
    int dutyCycle=(4*adc);
    
    ledcWrite(1, dutyCycle);
  
  
}
