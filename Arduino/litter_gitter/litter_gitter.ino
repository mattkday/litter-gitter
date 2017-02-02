#include <Wire.h>
#include <SharpIR.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define irL A0
#define irR A1
#define GateThresh 55
#define NormThresh 20
#define model 20150
#define SLAVE_ADDRESS 0x04
Adafruit_BNO055 bno = Adafruit_BNO055(55);
SharpIR sharpL(irL, 25, 93, model);
SharpIR sharpR(irR, 25, 93, model);


int x;
int y;
boolean isUpdated= false;
boolean searching = true;
boolean looking = true;
boolean start = true;
int cnt = 0;
int w = 1024/4;
int h = 768/4;
int leftTrig = w*0.2;
int rightTrig = w*0.8;
int side = 1;            //Left = 0    Right = 1
int speedPin_M1 = 5;     //M1 Speed Control
int speedPin_M2 = 6;     //M2 Speed Control
int directionPin_M1 = 4;     //M1 Direction Control
int directionPin_M2 = 7;     //M1 Direction Control

void setup(){
  Serial.begin(9600);
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // Initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // Define callbacks for i2c communication
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  // Initialize motor driver pins
  pinMode(speedPin_M1, OUTPUT);
  pinMode(speedPin_M2, OUTPUT);
  pinMode(directionPin_M1, OUTPUT);
  pinMode(directionPin_M2, OUTPUT);
  // ir sensor
  pinMode(irL, INPUT);
  pinMode(irR, INPUT);
}

void loop(){
  
  delay(100);
  //Serial.println("loop");
  cnt++;
  if (cnt ==8 && looking) {carStop(); Serial.println("Stop");}
  if (cnt>=20 && !searching) {
    searching = true;
    Serial.println("avoid obst again");
  }
  
  if (searching) {
    //stop motors
    //drive in pattern
    if (start) {
      start = false;
      advance();
      pointToDir(90);
    }
    
    
  }
}

void receiveEvent(int bytes) {
  int sentPoint = Wire.read();

  // Get object's points
  if (sentPoint == 0x01 && bytes > 1) {
    x = Wire.read();
    Serial.println("Object Found:");
    Serial.print(x);
  }
  else if (sentPoint == 0x02 && bytes > 1) {
    y = Wire.read();
    isUpdated = true;
    Serial.print(", ");
    Serial.println(y);
    if (looking) {
      searching = false;
      if (cnt > 20) {Serial.println("don't avoid obst");}
      cnt = 0;
      getObject();
    }
  }

  // Clear all remaining bytes
  while (Wire.available()) {
    Wire.read();
  }
}

void requestEvent() {
  //Wire.write((uint8_t *)&50, sizeof(50));
  Serial.println("request");
}

void getObject(){
  //stop motors
  //carStop();
  delay(800);
  if (y >= h*0.6) {
    //turn left
    Serial.println("Stop");
    carStop();
    //looking = false;
    //capture();
  }
  else if (x <= leftTrig) {
    //turn left
    Serial.println("Left");
    carTurnLeft(250,250);
    //delay(1000);
    //carStop();
  } 
  else if (x >= rightTrig ) {
    //turn right
    Serial.println("Right");
    carTurnRight(250,250);
    //delay(1000);
    //carStop();
  } 
  else if (x > leftTrig && x < rightTrig) {
    //drive forward
    Serial.println("Forward");
    //see if close to capture
    carAdvance(200,200);
    //delay(1000);
    //carStop();
  } 
  else {
    Serial.println("No Value");
  }
}

void advance(){
  int time = 0;
  while (time <= 1000) {
    int dist = sharpL.distance();
    if (dist > NormThresh) {
      carAdvance(175,175);
      delay(100);
      time += 100;
    }
    else {
      //hit wall    do something
    }
  }
}

int getOrient() {
  sensors_event_t event; 
  bno.getEvent(&event);
  Serial.print("X: ");
  Serial.println(event.orientation.x, 4);
  return event.orientation.x;
}

void pointToDir(int dir) {
  while (true) {
    int currDir = getOrient();
    int error = currDir - dir;
    
    if(error > 180) {
      error = error -360;
    }
    else if(error < -180) {
      error = error + 360;
    }
    
    if (error > 10) {
      //turn right
      carTurnRight(175,175);
    }
    else if (error < -10) {
      //turn left
      carTurnLeft(175,175);
    }
    else {
      carStop();
      break;
    }
  }
}

//-----MOTOR CONTROL-----//
void carStop(){                 //  Motor Stop
  digitalWrite(speedPin_M2,0); 
  digitalWrite(directionPin_M1,LOW);    
  digitalWrite(speedPin_M1,0);   
  digitalWrite(directionPin_M2,LOW);    
}   

void carBack(int leftSpeed,int rightSpeed){         //Move backward
  analogWrite (speedPin_M2,leftSpeed);              //PWM Speed Control
  digitalWrite(directionPin_M1,HIGH);    
  analogWrite (speedPin_M1,rightSpeed);    
  digitalWrite(directionPin_M2,HIGH);
} 

void carAdvance(int leftSpeed,int rightSpeed){       //Move forward
  analogWrite (speedPin_M2,leftSpeed);
  digitalWrite(directionPin_M1,LOW);   
  analogWrite (speedPin_M1,rightSpeed);    
  digitalWrite(directionPin_M2,LOW);
}

void carTurnLeft(int leftSpeed,int rightSpeed){      //Turn Left
  analogWrite (speedPin_M2,leftSpeed);
  digitalWrite(directionPin_M1,LOW);    
  analogWrite (speedPin_M1,rightSpeed);    
  digitalWrite(directionPin_M2,HIGH);
}
void carTurnRight(int leftSpeed,int rightSpeed){      //Turn Right
  analogWrite (speedPin_M2,leftSpeed);
  digitalWrite(directionPin_M1,HIGH);    
  analogWrite (speedPin_M1,rightSpeed);    
  digitalWrite(directionPin_M2,LOW);
}

