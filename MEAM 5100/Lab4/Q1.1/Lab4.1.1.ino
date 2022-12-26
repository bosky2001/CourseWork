void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(19,INPUT);
pinMode(4,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(4,HIGH);
Serial.println("The led is high frfr");
delay(10);
while(digitalRead(19)==1){
  Serial.println("The switch is high frfr");
  digitalWrite(4,LOW);
}
}
