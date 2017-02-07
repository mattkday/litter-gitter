#include <Wire.h>
#include <SharpIR.h>
#include <Servo.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define irL A0
#define irR A1
#define model 2080
#define SLAVE_ADDRESS 0x04
Adafruit_BNO055 bno = Adafruit_BNO055(55);
SharpIR sharpL(irL, 25, 93, model);
SharpIR sharpR(irR, 25, 93, model);
Servo myservo;


int x;
int y;
boolean isUpdated= false;
boolean searching = true;
boolean looking = true;
boolean start = true;
int cnt = 0;
int w = 1024/4;
int h = 768/4;
int gateUp = 0;
int gateDown = 90;
int leftTrig = w*0.2;
int rightTrig = w*0.8;
int GateThresh = 60;
int NormThresh = 40;
int distL = sharpL.distance();
int distR = sharpR.distance();
int side = 1;            //Left = 0    Right = 1
int PiPin = 2;
int speedPin_M1 = 5;     //M1 Speed Control
int speedPin_M2 = 6;     //M2 Speed Control
int directionPin_M1 = 4;     //M1 Direction Control
int directionPin_M2 = 7;     //M1 Direction Control

void setup(){
  Serial.begin(9600);
  
//  if(!bno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
  delay(1000);
  bno.setExtCrystalUse(true);
  myservo.attach(9);
  
  // Initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // Define callbacks for i2c communication
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  // Pi pin
  pinMode(PiPin, OUTPUT);
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
  Serial.println("loop");
  cnt++;
  if (cnt ==8 && looking && !searching) {
    carStop(); 
    Serial.println("Stop");
  }
  if (cnt>=20 && !searching) {
    searching = true;
    Serial.println("avoid obst again");
  }
  //searching
  if (searching) {
    piOff();
    distL = sharpL.distance();
    distR = sharpR.distance();
    //drive in pattern
    if (start) {
      start = false;
      piOff();
      advance();
      if (side == 1) {
        turnToLeft();
        side = 0;
      }
      else if(side == 0) {
        turnToRight();
        side = 1;
      }
      piOn();
      carAdvance(120,120);
      
    }
    else if (distL <= 0 || distR <= 0) {
      Serial.print("False readings: ");
      Serial.print(distL);
      Serial.println(distR);
    }
    else if (distL <= NormThresh || distR <= NormThresh) {
      piOff();
      Serial.println("dist");
      Serial.println(distL);
      Serial.println(distR);
      if (side == 1) {
        carStop();
        Serial.println("turn");
        turnToZero();
        advance();
        turnToLeft();
        carStop();
        side = 0;
      }
      else if (side == 0) {
        carStop();
        Serial.println("turn");
        turnToZero();
        advance();
        turnToRight();
        carStop();
        side = 1;
      }
      carAdvance(155,155);
      piOn();
      Serial.println("search");
    }
    else if (distL <= NormThresh+20 || distR <= NormThresh+20) {
      Serial.println("Slow Down");
      carAdvance(120,120);
    }
  }
  piOn();
}

void receiveEvent(int bytes) {
  int sentPoint = Wire.read();
  // Get object's points
  if (sentPoint == 0x01 && bytes > 1) {
    x = Wire.read();
    isUpdated = false;
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

void piOn() {
  if (looking == false) {
    looking = true;
    digitalWrite(PiPin, HIGH);
  }
}

void piOff() {
  if (looking == true) {
    looking = false;
    digitalWrite(PiPin, LOW);
  }
}

void requestEvent() {
  //Wire.write((uint8_t *)&50, sizeof(50));
  Serial.println("request");
}

void getObject(){
  //stop motors
  //carStop();
  int Speed = 110;
  delay(800);
  if (y >= h*0.6) {
    //turn left
    Serial.println("Stop");
    carStop();
    
    capture(); 
  }
  else if (x <= leftTrig) {
    //turn left
    Serial.println("Left");
    carTurnLeft(Speed,Speed);
    //delay(1000);
    //carStop();
  } 
  else if (x >= rightTrig ) {
    //turn right
    Serial.println("Right");
    carTurnRight(Speed,Speed);
    //delay(1000);
    //carStop();
  } 
  else if (x > leftTrig && x < rightTrig) {
    //drive forward
    Serial.println("Forward");
    //see if close to capture
    carAdvance(Speed,Speed);
    //delay(1000);
    //carStop();
  } 
  else {
    Serial.println("No Value");
  }
}

void capture(){
  piOff();
  searching = false;
  looking = false;
  myservo.write(gateDown);
  turnHome();
  while (distL >= GateThresh || distR >= GateThresh) {
    carAdvance(100,100);
  }
  carStop();
  turnToRight();
  while (distL >= GateThresh || distR >= GateThresh) {
    carAdvance(100,100);
  }
  carStop();
  myservo.write(gateUp);
  carBack(100,100);
  delay(1000);
  turnToZero();
  searching = true;
  looking = true;
  piOn();
}

void advance(){
  Serial.println("advance");
  int time = 0;
  if (time <= 800) {
    distL = sharpL.distance();
    distR = sharpR.distance();
    if (distL > NormThresh || distR > NormThresh) {
      carAdvance(155,155);
      delay(100);
      time += 100;
    }
    else {
      //hit wall    do something
      carStop();
    }
  }
}

int getOrient() {
  piOff();
  sensors_event_t event; 
  bno.getEvent(&event);
  Serial.print("X: ");
  Serial.println(event.orientation.x, 3);
  return event.orientation.x;
  piOn();
}

void turnHome() {
  piOff();
  while(true) {
    int currDir = getOrient();
    int Speed = 100;
    if (currDir < 140) {
      carTurnRight(Speed+50,Speed+50);
    }
    else if (currDir > 220) {
      carTurnLeft(Speed+50,Speed+50);
    }
    else if (currDir < 180) {
      carTurnRight(Speed,Speed);
    }
    else if (currDir > 180) {
      carTurnLeft(Speed,Speed);
    }
    else if (currDir <= 183 && currDir >= 177) {
      carStop();
      break;
    }
  }
}

void turnToZero() {
  piOff();
  while(true) {
    int currDir = getOrient();
    int Speed = 100;
    if (currDir < 320 && currDir >= 180) {
      carTurnRight(Speed+50,Speed+50);
    }
    else if (currDir > 40 && currDir < 180) {
      carTurnLeft(Speed+50,Speed+50);
    }
    else if (currDir < 360 && currDir >= 180) {
      carTurnRight(Speed,Speed);
    }
    else if (currDir > 0 && currDir < 180) {
      carTurnLeft(Speed,Speed);
    }
    else if (currDir >= 357 || currDir <= 3) {
      carStop();
      break;
    }
  }
}

void turnToRight() {
  piOff();
  while(true) {
    int currDir = getOrient();
    int Speed = 100;
    Serial.print(currDir);
    if (currDir < 50 || currDir >= 270) {
      carTurnRight(Speed+50,Speed+50);
    }
    else if (currDir > 130 && currDir < 270) {
      carTurnLeft(Speed+50,Speed+50);
    }
    else if (currDir < 90 || currDir >= 270) {
      carTurnRight(Speed,Speed);
    }
    else if (currDir > 90 && currDir < 270) {
      carTurnLeft(Speed,Speed);
    }
    else if (currDir <= 93 && currDir >= 87) {
      carStop();
      break;
    }
  }
}

void turnToLeft() {
  piOff();
  while(true) {
    int currDir = getOrient();
    int Speed = 100;
    if (currDir < 230 && currDir >= 90) {
      carTurnRight(Speed+50,Speed+50);
    }
    else if (currDir > 310 || currDir < 90) {
      carTurnLeft(Speed+50,Speed+50);
    }
    else if (currDir < 270 && currDir >= 90) {
      carTurnRight(Speed,Speed);
    }
    else if (currDir > 270 || currDir < 90) {
      carTurnLeft(Speed,Speed);
    }
    else if (currDir <= 273 && currDir >= 267) {
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

