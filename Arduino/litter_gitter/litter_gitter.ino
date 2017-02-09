#include <Wire.h>
#include <SharpIR.h>
#include <Servo.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define irL A0
#define irR A1
#define model 20150
#define SLAVE_ADDRESS 0x04
Adafruit_BNO055 bno = Adafruit_BNO055(55);
SharpIR sharpL(irL, 25, 93, model);
SharpIR sharpR(irR, 25, 93, model);
Servo myservo;

int x;
int y;
boolean searching = true;
boolean looking = true;
boolean start = true;
boolean Capture = false;
int cnt = 0;
int w = 1024/4;
int h = 768/4;
int gateUp = 0;
int gateDown = 90;
int leftTrig = w*0.4;
int rightTrig = w*0.6;
int vertTrig = h*0.8;
int GateThresh = 85;
int NormThresh = 55;
int SlowThresh = NormThresh + 25;
int GSpeed = 120;
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
  
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055
    Serial.print("Ooops, no BNO055 detected");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  // Attach the servo
  myservo.attach(9);
  // Initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // Define callbacks for i2c communication
  Wire.onReceive(receiveEvent);
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
  myservo.write(gateUp);
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
  if (Capture == true) {
    capture();
    Capture = false;
  }
  // searching
  if (searching) {
    piOff();
    distL = sharpL.distance();
    distR = sharpR.distance();
    //drive in pattern
    if (start) {
      start = false;
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
      // if @ a wall decide whether to go left or right
      piOff();
      Serial.println("dist");
      Serial.println(distL);
      Serial.println(distR);
      //turn, move forward, then turn again to search
      carStop(); 
      Serial.println("turn");
      turnToZero();
      advance();
      if (side == 1) {
        turnToLeft();
        side = 0;
      }
      else if (side == 0) {
        turnToRight();
        side = 1;
      }
      carStop();
      carAdvance(GSpeed,GSpeed);
      piOn();
      Serial.println("search");
    }
    else if (distL <= SlowThresh || distR <= SlowThresh) {
      Serial.println("Slow Down");
      carAdvance(100,100);
    }
  }
  piOn();
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
    Serial.print(", ");
    Serial.println(y);
    if (looking) {
      searching = false;
      if (cnt >= 20) {Serial.println("don't search");}
      cnt = 0;
      getObject();
    }
  }
  // Clear all remaining bytes
  while (Wire.available()) {
    Wire.read();
  }
}

void getObject(){
  int Speed = 120;
  delay(800);
  if (x <= leftTrig) {
    //turn left
    Serial.println("Left");
    carTurnLeft(Speed,Speed);
  } 
  else if (x >= rightTrig ) {
    //turn right
    Serial.println("Right");
    carTurnRight(Speed,Speed);
  } 
  else if (x > leftTrig && x < rightTrig) {
    //see if close to capture
    if (y >= vertTrig) {
      //stop and capture
      Serial.println("Stop and capture");
      carStop();
      Capture = true;
    }
     else {
      //drive forward
      Serial.println("Forward");
      carAdvance(Speed,Speed);
    }
  } 
  else {
    Serial.println("No Value");
  }
}

void capture() {
  piOff();
  searching = false;
  looking = false;
  myservo.write(gateDown);
  
  turnHome();
  Serial.println("turn home");
  distL = sharpL.distance();
  distR = sharpR.distance();
  Serial.println(distL);
  while (distL >= GateThresh || distR >= GateThresh) {
    carAdvance(100,100);
    distL = sharpL.distance();
    distR = sharpR.distance();
    Serial.println(distL);
  }
  carStop();
  Serial.println("turn right");
  turnToRight();
  distL = sharpL.distance();
  distR = sharpR.distance();
  while (distL >= GateThresh || distR >= GateThresh) {
    carAdvance(100,100);
    distL = sharpL.distance();
    distR = sharpR.distance();
  }
  carStop();
  myservo.write(gateUp);
  carBack(100,100);
  delay(1000);
  Serial.println("turn zero");
  turnToZero();
  searching = true;
  looking = true;
  start = true;
  side = 1;
  piOn();
}

void advance(){
  Serial.println("advance");
  int time = 0;
  while (time <= 500) {
    distL = sharpL.distance();
    distR = sharpR.distance();
    if (distL > NormThresh || distR > NormThresh) {
      carAdvance(GSpeed,GSpeed);
      delay(100);
      time += 100;
    }
    else {
      //hit wall    stop
      carStop();
    }
  }
}

void piOn() {
  // allow Pi to send points
  if (looking == false) {
    looking = true;
    digitalWrite(PiPin, HIGH);
  }
}

void piOff() {
  // don't allow Pi to send points
  if (looking == true) {
    looking = false;
    digitalWrite(PiPin, LOW);
  }
}

int getOrient() {
  piOff();
  // get the current direction
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
    int Speed = 130;
    if (currDir < 140) {
      carTurnRight(Speed+40,Speed+40);
    }
    else if (currDir > 220) {
      carTurnLeft(Speed+40,Speed+40);
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
    int Speed = 130;
    if (currDir < 320 && currDir >= 180) {
      carTurnRight(Speed+40,Speed+40);
    }
    else if (currDir > 40 && currDir < 180) {
      carTurnLeft(Speed+40,Speed+40);
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
    int Speed = 130;
    Serial.print(currDir);
    if (currDir < 50 || currDir >= 270) {
      carTurnRight(Speed+40,Speed+40);
    }
    else if (currDir > 130 && currDir < 270) {
      carTurnLeft(Speed+40,Speed+40);
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
    int Speed = 130;
    if (currDir < 230 && currDir >= 90) {
      carTurnRight(Speed+40,Speed+40);
    }
    else if (currDir > 310 || currDir < 90) {
      carTurnLeft(Speed+40,Speed+40);
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

