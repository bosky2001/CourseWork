#include <Wire.h>
#include <VL53L1X.h>
#define SONAR_NUM 1
#define PING_INTERVAL 33

VL53L1X sensor;

#define DistTrigger1 4
#define DistEcho1 5
#define DistTrigger2    //set pins according to availability for trigger and echo
#define DistEcho2   


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000); // allows 50ms for a measurement

  sensor.startContinuous(50); // starts measurement every 50ms

}
 

void loop()
{
   float rangeFront  //setting threshold for distances 
   int toCloseWall = 3; //closest range to wall
   int toFarWall= 5; //furthest range to wall
   int toCloseFront = 5; //closest range to front

   Main:
  rangeFront = sensor.read(); //values from ToF sensor
  rangeFront = rangeFront/10; 
  Serial.print(rangeFront);
  if (sensor.timeoutOccurred()) 
  { 
    Serial.print(" TIMEOUT"); 
  }

  Serial.println();

  Serial.print(rangeFront);          

  Serial.print(" Front");

  Serial.println();
   
   rangeWall = readRangeWall1(); // Read the sensor value by function call
   Serial.print(rangeWall);
   Serial.print("Wall");
   Serial.println();

  if(rangeFront < toCloseFront)

{
  delay(500);
  move_rev();
  Serial.print("Move Reverse");

right_turn(15); //right turn command with rate set to 15 for 90 deg turn
Serial.print("Right Turn");
Serial.println();
delay(800);

move_fwd();
left_turn(0.833); //left turn command with rate set to 0.833 for 5 deg turn
delay(1000);
goto Main;
}

if(rangeWall_left > toCloseWall && rangeWall < toFarWall)
{
  move_fwd();
  Serial.print("Move Forward");
  Serial.println();
  goto Main;
}

if(rangeWall_left < toCloseWall)
{
  delay(100);
  right_turn(0.833);
  delay(200);
  Serial.print("Turn Right");

  move_fwd();
  Serial.print("Move Forward");
  Serial.println();
  
  goto Main;
  }
  if(rangeWall_left > toFarWall)
  {
    delay(100);
    left_turn(0.833);
    Serial.print("Turn Left");
    delay(200);
    
    move_fwd();
    Serial.print("Drive Forward");
    Serial.println();
    goto Main;
  }


}  

  float readRangeWall1()
  {
    digitalWrite(DistTrigger, LOW);
    delayMicroseconds(2);
    digitalWrite(DistTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(DistTrigger, LOW);
    
    float duration = pulseIn(DistEcho, HIGH);  // in microseconds
    float range_Wall_left = duration * 0.0343 / 2.0;  // in centimeters
    
    return range_Wall_left;
}    
  

    
    return rangeFront;

  
  
    

  }
   


}
