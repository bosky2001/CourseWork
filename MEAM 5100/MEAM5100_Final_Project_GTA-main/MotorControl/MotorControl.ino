
#include<PID_v1.h>


#define DT1 8
#define DE1 3

#define DT2 20
#define DE2 21

#define ENC1 35 
#define ENC2 34
//int u1 = 3500;
//int u2 =2500;

double fwdrpm = 220;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

volatile int pos_i2 = 0;
volatile float velocity_2 = 0;
volatile long prevT_2 = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;

double Kp1=1.1, Ki1=1.0, Kd1=0.09;                   // Gains for the PID controller
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);    // making PID controller class

double Kp2=1.1, Ki2=1.0, Kd2=0.09;
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);


float readDistanceSensor(int DistTrigger, int DistEcho) {
   digitalWrite(DistTrigger, LOW);
   delayMicroseconds(2);
   digitalWrite(DistTrigger, HIGH);
   delayMicroseconds(10);
   digitalWrite(DistTrigger, LOW);
   float duration = pulseIn(DistEcho, HIGH);  // in microseconds
   float distance = duration * 0.0343 / 2.0;  // in centimeters
   return distance;
}

void readEncoder1(){
  // Read encoder 
  int b = digitalRead(ENC1);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}

void readEncoder2(){
  // Read encoder 
  int b = digitalRead(ENC2);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i2 = pos_i2 + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_2))/1.0e6;
  velocity_2 = increment/deltaT;
  prevT_2 = currT;
}

void move_fwd(int *f1, int *f2, double spd =fwdrpm){

   ledcWrite(0,0);
   ledcWrite(3,0);
   
   Setpoint1 = spd;
   Setpoint2 = spd;
   *f1 = 1;
   *f2 = 2;
   
  }

void move_rev(int *f3, int *f4, double spd =fwdrpm){

   ledcWrite(1,0);
   ledcWrite(2,0);
   
   Setpoint1 = spd;
   Setpoint2 = spd;
   *f3 = 0;
   *f4 = 3;
   
  }
void brake(int tim = 1000){
  ledcWrite(0,255);       // motor driver function brake setting
  ledcWrite(1,255);
  ledcWrite(2,255);
  ledcWrite(3,255);
  delay(tim);
}

void left_turn( float rate,double spd = fwdrpm){
    Setpoint1 = spd;
    Setpoint2 = spd- rate;  
    
    //brake();
}

void right_turn( int rate,double spd = fwdrpm ){
    Setpoint1 = spd-rate;
    Setpoint2 = spd;   
    
    //brake(); 
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);


  pinMode(41, OUTPUT);
  pinMode(42, OUTPUT);

  pinMode(35,INPUT);

  pinMode(34,INPUT);

  //pinMode(5,INPUT);
  //digitalWrite(4, LOW);
  //digitalWrite(5, LOW);

  ledcSetup(0,1000,8);
  ledcSetup(1,1000,8);

  ledcSetup(2,1000,8);
  ledcSetup(3,1000,8);

  pinMode(DT1, OUTPUT);
  pinMode(DE1, INPUT);

  pinMode(DT2, OUTPUT);
  pinMode(DE2, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(34),
                  readEncoder1,RISING);

  attachInterrupt(digitalPinToInterrupt(35),
                  readEncoder2,RISING);

  //myPID1.SetOutputLimits(0,4095);  // setting PID control output limits
  //myPID2.SetOutputLimits(0,4095);  
                
  myPID1.SetMode(AUTOMATIC); 
  myPID2.SetMode(AUTOMATIC);

  //ledcWrite(1,3500);
  //ledcWrite(2,3500);
}

void loop() {
  // put your main code here, to run repeatedly:
  //pinMode(4,OUTPUT);
  //pinMode(6,OUTPUT);

  ledcAttachPin(4,0);
  ledcAttachPin(6,1);
  //ledcWrite(1,4095);
  

  ledcAttachPin(41,2);
  ledcAttachPin(42,3);
  
  
  //digitalWrite(4,HIGH);

  int pos = 0;
  float vel1 = 0;
  float vel2=0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  vel1 = velocity_i;         // finding velocity from readencoder function using method 2
  vel2 = velocity_2;
  interrupts();

  float v1 = ((vel1)/20.0)*60.0;
  float v2 = ((vel2)/20.0)*60.0;

  /* v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;          // applying a low-pass filter to the reading to account for noisy values
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;  */
  
  int d1, d2;
  
  
  
  
  Input1 = v1Filt;
  Input2 = v2Filt;
  myPID1.Compute();
  myPID2.Compute();
  
  int u1= Output1;   // setting the control signal to the motors
  int u2= Output2;

  ledcWrite(d1,u1);
  ledcWrite(d2,u2);
  
  //Serial.print(v1);             //printing the deets
  //Serial.print(" ");
  Serial.print(Setpoint1);
  Serial.print(" ");
  Serial.print(Setpoint2);
  Serial.print(" ");
  
  Serial.print(v1);
  Serial.print(" ");

  Serial.print(v2);
  Serial.print(" ");
  Serial.print(Output1);
  Serial.print(" ");
  Serial.print(Output2);
  Serial.println();
  
  delay(10);
  
}
